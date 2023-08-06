/*!
  \file main.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

// Standard C++ library
#include <array>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <utility>
// CLI11
#include "CLI/CLI.hpp"
// Zisc
#include "zisc/memory/alloc_free_resource.hpp"
// Zivc
#include "zivc/zivc.hpp"
// Nanairo Cmd
#include "cli.hpp"
#include "ldr_image.hpp"
#include "png_writer.hpp"

zivc::uint8b to8bitColor(const double v) noexcept
{
  constexpr double minv = 0.0;
  constexpr double maxv = 255.0;
  const double p = std::clamp(256.0 * v, minv, maxv);
  return static_cast<zivc::uint8b>(p);
}

void render([[maybe_unused]] const cmd::CliOptions& options, cmd::LdrImage* output) noexcept
{
  using zivc::uint8b;
  const auto max_w = static_cast<double>(output->width());
  const auto max_h = static_cast<double>(output->height());
  for (std::size_t h = 0; h < output->height(); ++h) {
    const uint8b green = to8bitColor(static_cast<double>(h) / max_h);
    for (std::size_t w = 0; w < output->width(); ++w) {
      constexpr uint8b max_value = 255;
      const uint8b red = to8bitColor(static_cast<double>(w) / max_w);
      zivc::cl::uchar4& p = (*output)[w + h * output->width()];
      p.x = red;
      p.y = green;
      p.z = 0;
      p.w = max_value;
    }
  }
}

void saveImage(const std::size_t frame, const cmd::LdrImage& image, const cmd::PngWriter& writer)
{
  constexpr std::size_t max_length = 256;
  std::array<char, max_length> file_name{};
  std::snprintf(file_name.data(), file_name.size(), "%06d.png", static_cast<int>(frame));
  std::ofstream image_file{file_name.data(), std::ios_base::binary};
  writer.write(image, &image_file);
}

int main(const int argc, const char* const* const argv)
{
  // Process command line arguments
  cmd::CliOptions options{};
  {
    std::unique_ptr cli_parser = cmd::createCommandLineParser(&options);
    CLI11_PARSE(*cli_parser, argc, argv);
  }

  //
  const std::unique_ptr mem_resource = std::make_unique<zisc::AllocFreeResource>();

  {
    // Create an LDR image for output
    std::unique_ptr ldr_image = std::make_unique<cmd::LdrImage>(mem_resource.get());
    ldr_image->initialize(options.image_width_, options.image_height_);

    //
    std::unique_ptr png_writer = std::make_unique<cmd::PngWriter>(mem_resource.get());

    for (std::size_t frame = options.min_frame_; frame < options.max_frame_; ++frame) {
      render(options, ldr_image.get());
      png_writer->initialize();
      saveImage(frame, *ldr_image, *png_writer);
    }
  }

  return EXIT_SUCCESS;
}
