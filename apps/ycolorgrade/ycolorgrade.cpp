//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
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

#include <yocto/yocto_cli.h>
#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_sceneio.h>
#include <yocto_colorgrade/yocto_colorgrade.h>
#include <yocto_gui/yocto_glview.h>
using namespace yocto;

void run_offline(
    const string& filename, const string& output, const grade_params& params) {
  // load
  auto error = string{};
  auto image = image_data{};
  if (!load_image(filename, image, error)) print_fatal(error);

  // hack to convert to srgb on input since we handle corrections outselves
  image.linear = false;

  // corrections
  auto graded = grade_image(image, params);

  // save
  if (!save_image(output, graded, error)) print_fatal(error);
}

void run_interactively(
    const string& filename, const string& output, const grade_params& params_) {
  // copy params
  auto params = params_;

  // load
  auto error = string{};
  auto image = image_data{};
  if (!load_image(filename, image, error)) print_fatal(error);

  // hack to convert to srgb on input since we handle corrections outselves
  image.linear = false;

  // display image
  auto display = grade_image(image, params);

  // opengl image
  auto glimage  = glimage_state{};
  auto glparams = glimage_params{};

  // top level combo
  auto names    = vector<string>{filename};
  auto selected = 0;

  // callbacks
  auto callbacks    = glwindow_callbacks{};
  callbacks.init_cb = [&](const glinput_state& input) {
    init_image(glimage);
    set_image(glimage, display);
  };
  callbacks.clear_cb = [&](const glinput_state& input) {
    clear_image(glimage);
  };
  callbacks.draw_cb = [&](const glinput_state& input) {
    glparams.window                           = input.window_size;
    glparams.framebuffer                      = input.framebuffer_viewport;
    std::tie(glparams.center, glparams.scale) = camera_imview(glparams.center,
        glparams.scale, {image.width, image.height}, glparams.window,
        glparams.fit);
    draw_image(glimage, glparams);
  };
  callbacks.widgets_cb = [&](const glinput_state& input) {
    draw_glcombobox("name", selected, names);
    if (begin_glheader("colorgrade")) {
      auto edited = 0;
      edited += draw_glslider("exposure", params.exposure, -5, 5);
      edited += draw_glcheckbox("filmic", params.filmic);
      edited += draw_glcheckbox("srgb", params.srgb);
      edited += draw_glcoloredit("tint", params.tint);
      edited += draw_glslider("contrast", params.contrast, 0, 1);
      edited += draw_glslider("saturation", params.saturation, 0, 1);
      edited += draw_glslider("vignette", params.vignette, 0, 1);
      edited += draw_glslider("grain", params.grain, 0, 1);
      edited += draw_glslider("mosaic", params.mosaic, 0, 64);
      edited += draw_glslider("grid", params.grid, 0, 64);
      edited += draw_glcheckbox("grey", params.grey);
      edited += draw_glcheckbox("sketch", params.sketch);
      edited += draw_glcheckbox("vhs", params.vhs);
      edited += draw_glcheckbox("clash", params.clash);
      edited += draw_glcheckbox("negative", params.negative);
      edited += draw_glslider("mirror", params.mirror, 0, 100);
      edited += draw_glcheckbox("stippling", params.stippling);
      end_glheader();
      if (edited) {
        display = grade_image(image, params);
        set_image(glimage, display);
      }
    }
    // draw_image_inspector(input, image, display, glparams);
  };

  // run ui
  run_ui({1280 + 320, 720}, "yicolorgrade", callbacks);
}

void run(const vector<string>& args) {
  // command line parameters
  auto params      = grade_params{};
  auto output      = "out.png"s;
  auto filename    = "img.hdr"s;
  auto interactive = false;

  // parse command line
  auto error = string{};
  auto cli   = make_cli("ycolorgrade", "Transform images");
  add_option(cli, "image", filename, "Input image filename");
  add_option(cli, "output", output, "Output image filename");
  add_option(cli, "interactive", interactive, "Run interactively");
  add_option(cli, "exposure", params.exposure, "Tonemap exposure");
  add_option(cli, "filmic", params.filmic, "Tonemap uses filmic curve");
  add_option(cli, "saturation", params.saturation, "Grade saturation");
  add_option(cli, "contrast", params.contrast, "Grade contrast");
  add_option(cli, "tint-red", params.tint.x, "Grade red tint");
  add_option(cli, "tint-green", params.tint.y, "Grade green tint");
  add_option(cli, "tint-blue", params.tint.z, "Grade blue tint");
  add_option(cli, "vignette", params.vignette, "Vignette radius");
  add_option(cli, "grain", params.grain, "Grain strength");
  add_option(cli, "mosaic", params.mosaic, "Mosaic size (pixels)");
  add_option(cli, "grid", params.grid, "Grid size (pixels)");
  add_option(cli, "grey", params.grey, "Grey");
  add_option(cli, "sketch", params.sketch, "Sketch");
  add_option(cli, "vhs", params.vhs, "VHS");
  add_option(cli, "clash", params.clash, "Attribute clash");
  add_option(cli, "negative", params.negative, "Negative");
  add_option(cli, "mirror", params.mirror, "Mirror (percentage width)");
  add_option(cli, "stippling", params.stippling, "Stippling");
  add_option(cli, "nPoints", params.nPoints, "nPoints stippling");
  add_option(cli, "threshold", params.threshold, "threshold stippling");
  add_option(cli, "resolution", params.resolution, "resolution stippling");
  add_option(cli, "iterations", params.iterations, "iterations stippling");
  add_option(cli, "rMin", params.rMin, "rMin stippling");
  add_option(cli, "rMax", params.rMax, "rMax stippling");
  if (!parse_cli(cli, args, error)) print_fatal(error);

  if (interactive) {
    run_interactively(filename, output, params);
  } else {
    run_offline(filename, output, params);
  }
}

int main(int argc, const char* argv[]) { run(make_cli_args(argc, argv)); }
