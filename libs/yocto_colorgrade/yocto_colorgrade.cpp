//
// Implementation for Yocto/Grade.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "yocto_colorgrade.h"

#include <time.h>
#include <yocto/yocto_color.h>
#include <yocto/yocto_sampling.h>

#include <map>
using namespace std;

#define CAMPOW 1.0f          // adjust input gamma
#define LOWREZ 4.0f          // chunky pixel size
#define CFULL 1.0f           // color brightnesses
#define CHALF 0.8431372549f  // color brightnesses
#define DITHER 1.0f          // dither amount, 0 to 1

typedef struct Point {
  float X;
  float Y;

  bool operator<(const Point& other) const {
    return (X < other.X || (X == other.X && Y < other.Y));
  }
};

typedef struct rectangle {
  Point min;
  Point max;
};

typedef struct kdNode {
  Point   p;
  bool    splitByX;
  kdNode* left;
  kdNode* right;
};

typedef struct kdTree {
  kdNode*   root;
  rectangle bounds;
};

// getDist computes the squared euclidean distance to another Point.
float getDist(Point p, Point q) {
  auto dx = p.X - q.X;
  auto dy = p.Y - q.Y;
  return dx * dx + dy * dy;
}

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto {

vec3f toneMapping(vec3f c, float exposure, bool filmic, bool srgb) {
  c *= pow(2, exposure);
  if (filmic) {
    c *= 0.6;
    c = (pow(c, 2) * 2.51 + c * 0.03) / (pow(c, 2) * 2.43 + c * 0.59 + 0.14);
  }
  if (srgb) {
    c = pow(c, 1 / 2.2);
  }
  c = clamp(c, 0, 1);
  return c;
}

vec3f colorTint(vec3f c, vec3f tint) { return c * tint; }

vec3f saturation(vec3f c, float saturation) {
  auto g = (c.x + c.y + c.z) / 3;
  c      = g + (c - g) * (saturation * 2);
  return c;
}

vec3f contrast_c(vec3f c, float contrast) { return gain(c, 1 - contrast); }

vec3f vignette(vec3f c, float vignette, int n, int width, int height) {
  auto  vr   = 1 - vignette;
  int   i    = n % width;
  int   j    = floor(n / width);
  vec2f ij   = {i, j};
  vec2f size = {width, height};
  return c *
         (1 - smoothstep(vr, 2 * vr, length(ij - size / 2) / length(size / 2)));
}

vec3f filmGrain(vec3f c, rng_state& rng, float grain) {
  return c + (rand1f(rng) - 0.5) * grain;
}

void mosaic(color_image& image, int mosaic) {
  for (int i = 0; i < image.width; i++) {
    for (int j = 0; j < image.height; j++) {
      auto c        = xyz(image[{i, j}]);
      c             = xyz(image[{i - i % mosaic, j - j % mosaic}]);
      image[{i, j}] = vec4f{c.x, c.y, c.z, image[{i, j}].w};
    }
  }
}

void grid(color_image& image, int grid) {
  for (int i = 0; i < image.width; i++) {
    for (int j = 0; j < image.height; j++) {
      auto c = xyz(image[{i, j}]);
      if (0 == i % grid || 0 == j % grid) {
        c *= 0.5;
      }
      image[{i, j}] = vec4f{c.x, c.y, c.z, image[{i, j}].w};
    }
  }
}

void sketch(color_image& image) {
  for (auto& pixel : image.pixels) {
    float r = sin(pixel.x * 6.28 * .9);
    auto  c = smoothstep(0, 0.8f, 1 - pow(r, 6));
    pixel   = vec4f{c, c, c, pixel.w};
  }
}

void vhs(color_image& image) {
  auto image_clone = image;
  for (int i = 0; i < image.width; i++) {
    for (int j = 0; j < image.height; j++) {
      float x  = static_cast<float>(i) / image.width;
      float y  = static_cast<float>(j) / image.height;
      vec2f uv = vec2f{x, y};
      float d  = length(uv - vec2f{0.5, 0.5});

      float blur = 0.01;

      float x_b   = uv.x + blur;
      float x_b_l = uv.x - blur;
      vec3f c     = {eval_image(image_clone, vec2f{x_b, uv.y}).x,
          eval_image(image_clone, uv).y,
          eval_image(image_clone, vec2f{x_b_l, uv.y}).z};

      // scanline
      float scanline = sin(uv.y * 400) * 0.08;
      c -= scanline;

      // vignette
      c *= 1.0 - d * 0.5;

      image[{i, j}] = vec4f{c.x, c.y, c.z, image[{i, j}].w};
    }
  }
}

vec4f smap(vec3f c) {
  c = pow(c, vec3f{CAMPOW, CAMPOW, CAMPOW});
  if ((c.x > CHALF) || (c.y > CHALF) || (c.z > CHALF))  // bright?
  {
    return vec4f{c.x, c.y, c.z, 1.0};
  } else {
    vec3f min_c = min((c / CHALF), vec3f{1.0, 1.0, 1.0});
    return vec4f{min_c.x, min_c.y, min_c.z, 0.0};
  }
}

vec4f bmap(vec3f c) {
  c = pow(c, vec3f{CAMPOW, CAMPOW, CAMPOW});
  if ((c.x > CHALF) || (c.y > CHALF) || (c.z > CHALF))  // bright?
  {
    vec3f floor_c = vec3f{
        floor(c.x + 0.5f), floor(c.y + 0.5f), floor(c.z + 0.5f)};
    return vec4f{floor_c.x, floor_c.y, floor_c.z, 1.0};
  } else {
    vec3f floor_c = min(
        vec3f{floor(c.x / CHALF + 0.5f), floor(c.y / CHALF + 0.5f),
            floor(c.z / CHALF + 0.5f)},
        vec3f{1.0, 1.0, 1.0});
    return vec4f{floor_c.x, floor_c.y, floor_c.z, 0};
  }
}

vec3f fmap(vec4f c) {
  if (c.w >= 0.5) {
    return xyz(c) * vec3f{CFULL, CFULL, CFULL};
  } else {
    return xyz(c) * vec3f{CHALF, CHALF, CHALF};
  }
}

void zxspectrum_clash(color_image& image) {
  auto image_clone = image;
  for (int i = 0; i < image.width; i++) {
    for (int j = 0; j < image.height; j++) {
      vec2f pv = vec2f{floor(i / LOWREZ), floor(j / LOWREZ)};
      vec2f bv = vec2f{floor(pv.x / 8.0f) * 8.0f, floor(pv.y / 8.0f) * 8.0f};
      vec2f sv = vec2f{
          floor(image.width / LOWREZ), floor(image.height / LOWREZ)};

      vec4f min_cs = vec4f{1.0, 1.0, 1.0, 1.0};
      vec4f max_cs = vec4f{0.0, 0.0, 0.0, 0.0};
      float bright = 0.0;

      for (int py = 1; py < 8; py++) {
        for (int px = 0; px < 8; px++) {
          vec4f cs = bmap(xyz(
              eval_image(image_clone, (bv + vec2f{px + 0.f, py + 0.f}) / sv)));
          bright += cs.w;
          min_cs = min(min_cs, cs);
          max_cs = max(max_cs, cs);
        }
      }

      if (bright >= 24.0) {
        bright = 1.0;
      } else {
        bright = 0.0;
      }

      if (xyz(max_cs) == xyz(min_cs)) {
        min_cs = vec4f{0.0, 0.0, 0.0, min_cs.w};
      }

      if (max_cs.x == 0 && max_cs.y == 0 && max_cs.z == 0) {
        bright = 0.0;
        max_cs = vec4f{0.0, 0.0, 1.0, max_cs.w};
        min_cs = vec4f{0.0, 0.0, 0.0, min_cs.w};
      }

      vec3f c1 = fmap(vec4f{max_cs.x, max_cs.y, max_cs.z, bright});
      vec3f c2 = fmap(vec4f{min_cs.x, min_cs.y, min_cs.z, bright});

      vec3f cs = xyz(eval_image(image_clone, pv / sv));

      vec3f d  = (cs + cs) - (c1 + c2);
      float dd = d.x + d.y + d.z;

      if (fmod(pv.x + pv.y, 2.0f) == 1.0) {
        image[{i, j}] = vec4f{dd >= -(DITHER * 0.5) ? c1.x : c2.x,
            dd >= -(DITHER * 0.5) ? c1.y : c2.y,
            dd >= -(DITHER * 0.5) ? c1.z : c2.z, 1.0};
      } else {
        image[{i, j}] = vec4f{dd >= (DITHER * 0.5) ? c1.x : c2.x,
            dd >= (DITHER * 0.5) ? c1.y : c2.y,
            dd >= (DITHER * 0.5) ? c1.z : c2.z, 1.0};
      }
    }
  }
}

void negative(color_image& image) {
  for (auto& pixel : image.pixels) {
    auto c = vec3f{1, 1, 1} - xyz(pixel);
    pixel  = vec4f{c.x, c.y, c.z, pixel.w};
  }
}

void mirror(color_image& image, int start) {
  start = image.width / 100 * start;
  for (int i = 0; i < image.width - start; i++) {
    for (int j = 0; j < image.height; j++) {
      image[{start + i, j}] = image[{abs(start - i) % image.width, j}];
    }
  }
}

void toGray(color_image& image) {
  for (auto& pixel : image.pixels) {
    auto c = pixel.x * 0.2989f + pixel.y * 0.5870f + pixel.z * 0.1140f;
    pixel  = vec4f{c, c, c, pixel.w};
  }
}

vector<Point> importanceSampling(
    int n, const color_image& gray, rng_state& rng, int threshold) {
  map<int, vector<Point>> hist;
  vector<Point>           pts;
  vector<int>             roulette;

  for (int i = 0; i < gray.width; i++) {
    for (int j = 0; j < gray.height; j++) {
      auto intensity = xyz(gray[{i, j}]).x * 256;
      if (intensity <= threshold) {
        hist[intensity].insert(
            hist[intensity].begin(), Point{(float)i, (float)j});
      }
    }
  }

  roulette.insert(roulette.begin(), 256);

  for (int i = 1; i < threshold + 1; i++) {
    roulette.insert(
        roulette.begin() + i, roulette[i - 1] + (256 - i) * hist[i].size());
  }

  auto s_roulette = roulette;
  sort(s_roulette.begin(), s_roulette.end());

  for (int i = 0; i < n; i++) {
    int bin;
    do {
      auto ball = rand1i(rng, roulette[roulette.size() - 1]);
      bin       = upper_bound(s_roulette.begin(), s_roulette.end(), ball) -
            s_roulette.begin();
    } while (hist[bin].size() == 0);
    auto p = hist[bin][rand1i(rng, hist[bin].size())];
    p.X += rand1f(rng);
    p.Y += rand1f(rng);
    pts.insert(pts.begin() + i, p);
  }
  return pts;
}

vector<vector<float>> makePDF(const color_image& gray) {
  vector<vector<float>> pdf(gray.width, vector<float>(gray.height));
  for (int i = 0; i < gray.width; i++) {
    for (int j = 0; j < gray.height; j++) {
      pdf[i][j] = 1.0 - gray[{i, j}].y;
    }
  }
  return pdf;
}

kdNode* split(vector<Point> pts, bool splitByX) {
  if (pts.size() == 0) {
    return NULL;
  }
  if (pts.size() == 1) {
    return new kdNode{pts[0], splitByX, NULL, NULL};
  }

  if (splitByX) {
    sort(pts.begin(), pts.end(), [](Point p, Point q) { return p.X < q.X; });
  } else {
    sort(pts.begin(), pts.end(), [](Point p, Point q) { return p.Y < q.Y; });
  }

  float         med = pts.size() / 2;
  vector<Point> left_pts(pts.begin(), pts.begin() + med);
  vector<Point> right_pts(pts.begin() + med, pts.end());

  auto result = new kdNode{pts[med], splitByX, split(left_pts, !splitByX),
      split(right_pts, !splitByX)};
  return result;
}

kdTree makeKdTree(vector<Point> pts, rectangle bounds) {
  return {
      split(pts, true),
      bounds,
  };
}

tuple<Point, float> search(
    kdNode* node, Point target, rectangle r, float maxDistSqd) {
  if (node == NULL) {
    return {Point{}, numeric_limits<float>::infinity()};
  }

  bool      targetInLeft;
  rectangle leftBox, rightBox;
  kdNode *  nearestNode, *furthestNode;
  rectangle nearestBox, furthestBox;

  if (node->splitByX) {
    leftBox      = {r.min, Point{node->p.X, r.max.Y}};
    rightBox     = {Point{node->p.X, r.min.Y}, r.max};
    targetInLeft = target.X <= node->p.X;
  } else {
    leftBox      = {r.min, Point{r.max.X, node->p.Y}};
    rightBox     = {Point{r.min.X, node->p.Y}, r.max};
    targetInLeft = target.Y <= node->p.Y;
  }

  if (targetInLeft) {
    nearestNode  = node->left;
    nearestBox   = leftBox;
    furthestNode = node->right;
    furthestBox  = rightBox;
  } else {
    nearestNode  = node->right;
    nearestBox   = rightBox;
    furthestNode = node->left;
    furthestBox  = leftBox;
  }

  auto [nearest, distSqd] = search(nearestNode, target, nearestBox, maxDistSqd);

  if (distSqd < maxDistSqd) {
    maxDistSqd = distSqd;
  }

  float d;

  if (node->splitByX) {
    d = node->p.X - target.X;
  } else {
    d = node->p.Y - target.Y;
  }
  d *= d;

  if (d > maxDistSqd) {
    return {nearest, distSqd};
  }

  d = getDist(node->p, target);

  if (d < distSqd) {
    nearest    = node->p;
    distSqd    = d;
    maxDistSqd = distSqd;
  }

  auto [tmpNearest, tmpSqd] = search(
      furthestNode, target, furthestBox, maxDistSqd);

  if (tmpSqd < distSqd) {
    nearest = tmpNearest;
    distSqd = tmpSqd;
  }
  return {nearest, distSqd};
}

Point findNearestNeighbour(Point p, kdTree t) {
  auto [nearest, distSqd] = search(
      t.root, p, t.bounds, numeric_limits<float>::infinity());
  return nearest;
}

tuple<vector<Point>, vector<float>> getCentroids(vector<Point> sites,
    vector<vector<float>> pdf, rectangle bounds, float step) {
  map<Point, Point> siteCentroids;
  map<Point, float> siteIntensities;
  map<Point, float> siteNPoints;

  for (auto p : sites) {
    siteCentroids[p] = Point{};
  }

  auto kd = makeKdTree(sites, bounds);

  for (int i = bounds.min.X; i < bounds.max.X; i += step) {
    for (int j = bounds.min.Y; j < bounds.max.Y; j += step) {
      auto  p        = Point{(float)i, (float)j};
      auto  best     = findNearestNeighbour(p, kd);
      auto  w        = pdf[int(i)][int(j)];
      Point centroid = siteCentroids[best];
      centroid.X += w * p.X;
      centroid.Y += w * p.Y;
      siteCentroids[best] = centroid;
      siteIntensities[best] += w;
      siteNPoints[best]++;
    }
  }

  vector<Point> centroids;
  vector<float> densities;
  int           i = 0;

  for (const auto& [site, density] : siteIntensities) {
    auto centroid = siteCentroids[site];
    centroid.X /= density;
    centroid.Y /= density;
    centroids.insert(centroids.begin() + i, centroid);
    densities.insert(
        densities.begin() + i, siteIntensities[site] / siteNPoints[site]);
    i++;
  }

  return {centroids, densities};
}

vector<float> rescaleFloat(vector<float> floats, float newMin, float newMax) {
  vector<float> rescaled;
  auto          oldMin = floats[0];
  for (auto n : floats) {
    if (n < oldMin) {
      oldMin = n;
    }
  }
  for (int i = 0; i < floats.size(); i++) {
    rescaled.insert(
        rescaled.begin() + i, newMin + (newMax - newMin) * floats[i] / oldMin);
  }
  return rescaled;
}

void drawCircle(color_image& image, int start_X, int start_Y, int r) {
  for (int i = max(start_X - r, 0); i <= min(start_X + r, image.width - 1);
       i++) {
    for (int j = max(start_Y - r, 0); j <= min(start_Y + r, image.height - 1);
         j++) {
      if ((i - start_X) * (i - start_X) + (j - start_Y) * (j - start_Y) <=
          r * r) {
        image[{i, j}] = vec4f{0, 0, 0, 1};
      }
    }
  }
}

void stippling(color_image& image, rng_state& rng, int nPoints, int threshold,
    float resolution, int iterations, float rMin, float rMax) {
  auto gray = image;
  toGray(gray);
  auto new_image = make_image(gray.width, gray.height, false);
  for (auto& pixel : new_image.pixels) pixel = vec4f{1, 1, 1, 1};
  auto bounds = rectangle{
      Point{0, 0}, Point{(float)gray.width, (float)gray.height}};
  int  x      = nPoints;
  auto points = importanceSampling(nPoints, gray, rng, threshold);
  auto pdf    = makePDF(gray);
  auto step   = 1 / resolution;
  auto [stipples, densities] = getCentroids(points, pdf, bounds, step);
  for (int i = 1; i < iterations - 1; i++) {
    auto [new_stipples, new_densities] = getCentroids(
        stipples, pdf, bounds, step);
    stipples  = new_stipples;
    densities = new_densities;
  }
  auto radiuses = rescaleFloat(densities, rMin, rMax);
  for (int i = 0; i < stipples.size(); i++) {
    drawCircle(new_image, stipples[i].X, stipples[i].Y, radiuses[i]);
  }
  image = new_image;
}

color_image grade_image(const color_image& image, const grade_params& params) {
  auto graded = image;
  int  n      = 0;
  auto rng    = make_rng(time(NULL));
  for (auto pixel : graded.pixels) {
    auto c = xyz(pixel);
    c      = toneMapping(c, params.exposure, params.filmic, params.srgb);
    c      = colorTint(c, params.tint);
    c      = saturation(c, params.saturation);
    c      = contrast(c, params.contrast);
    c      = vignette(c, params.vignette, n, graded.width, graded.height);
    c      = filmGrain(c, rng, params.grain);
    graded.pixels[n] = vec4f{c.x, c.y, c.z, pixel.w};
    n++;
  }
  if (params.mosaic != 0) mosaic(graded, params.mosaic);
  if (params.grid != 0) grid(graded, params.grid);

  // EXTRA
  if (params.grey) toGray(graded);
  if (params.sketch) sketch(graded);
  if (params.vhs) vhs(graded);
  if (params.clash) zxspectrum_clash(graded);
  if (params.negative) negative(graded);
  if (params.mirror != 0) mirror(graded, params.mirror);
  if (params.stippling)
    stippling(graded, rng, params.nPoints, params.threshold, params.resolution,
        params.iterations, params.rMin, params.rMax);

  return graded;
}

}  // namespace yocto