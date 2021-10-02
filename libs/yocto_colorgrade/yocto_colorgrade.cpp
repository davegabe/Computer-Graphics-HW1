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

#include <yocto/yocto_color.h>
#include <yocto/yocto_sampling.h>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto {

void toneMapping(vec4f& c, float exposure, bool filmic, bool srgb) {
  // Exposure compensation
  c *= pow(2, exposure);
  // Filmic correction
  if (filmic) {
    c *= 0.6;
    c = (pow(c, 2) * 2.51 + c * 0.03) / (pow(c, 2) * 2.43 + c * 0.59 + 0.14);
  }
  // Srgb color space
  if (srgb) {
    c = pow(c, 1 / 2.2);
  }
  // Clamp result
  c = max(min(c, 1), 0);
}

void colorTint(vec4f& c, vec3f tint) {
  vec4f tintA = {tint.x, tint.y, tint.z, 1};
  c           = c * tintA;
}

void saturation(vec4f& c, float saturation) {
  auto g = (c.x + c.y + c.z) / 3;
  c      = g + (c - g) * (saturation * 2);
}

void contrast(vec4f& c, float contrast) { c = gain(c, 1 - contrast); }

void vignette(vec4f& c, float vignette, int n, int width, int height) {
  auto  vr   = 1 - vignette;
  int   i    = n % width;
  int   j    = floor(n / width);
  vec2f ij   = {i, j};
  vec2f size = {width, height};
  c          *= (1 - smoothstep(vr, 2 * vr,
                   length(ij - size / 2) / length(size / 2)));  //!!!!!!!!!
}

void filmGrain(vec4f& c, rng_state& rng, float grain) {
  c += (rand1f(rng) - 0.5) * grain;
}

void mosaic(vec4f& c, const color_image& image, int mosaic, int n, int width,
    int height) {
  if (mosaic != 0) {
    int i = n % width;
    int j = floor(n / width);
    c     = image[{i - i % mosaic, j - j % mosaic}];
  }
}

void grid(vec4f& c, const color_image& image, int grid, int n, int width,
    int height) {
  if (grid != 0) {
    int i = n % width;
    int j = floor(n / width);
    if (0 == i % grid || 0 == j % grid) {
      c *= 0.5;
    }
  }
}

color_image grade_image(const color_image& image, const grade_params& params) {
  auto graded = image;
  int  n      = 0;
  auto rng    = make_rng(172784);
  for (auto& pixel : graded.pixels) {
    toneMapping(pixel, params.exposure, params.filmic, params.srgb);
    colorTint(pixel, params.tint);
    saturation(pixel, params.saturation);
    contrast(pixel, params.contrast);
    vignette(pixel, params.vignette, n, graded.width, graded.height);
    filmGrain(pixel, rng, params.grain);
    mosaic(pixel, graded, params.mosaic, n, graded.width, graded.height);
    n++;
  }
  n = 0;
  for (auto& pixel : graded.pixels) {
    grid(pixel, graded, params.grid, n, graded.width, graded.height);
    n++;
  }
  return graded;
}

}  // namespace yocto