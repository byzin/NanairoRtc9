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
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
// CLI11
#include "CLI/CLI.hpp"
// Zisc
#include "zisc/memory/alloc_free_resource.hpp"
// Zivc
#include "zivc/zivc.hpp"
// Nanairo Cmd
#include "cli.hpp"
#include "gltf_scene.hpp"
#include "ldr_image.hpp"
#include "png_writer.hpp"
#include "renderer.hpp"

zivc::uint8b to8bitColor(const double v) noexcept
{
  constexpr double minv = 0.0;
  constexpr double maxv = 255.0;
  const double p = std::clamp(256.0 * v, minv, maxv);
  return static_cast<zivc::uint8b>(p);
}

void loadGltfScene(const std::string_view gltf_scene_path, nanairo::GltfScene* scene) noexcept
{
  std::ifstream data{gltf_scene_path.data(), std::ios_base::binary};
  if (!data.is_open()) {
    std::cerr << "  Loading gltf scene '" << gltf_scene_path << "' failed." << std::endl;
  }

  scene->loadBinary(data);
}

void saveImage(const std::size_t frame, const nanairo::LdrImage& image, const nanairo::PngWriter& writer)
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
  nanairo::CliOptions options{};
  {
    std::unique_ptr cli_parser = nanairo::createCommandLineParser(&options);
    CLI11_PARSE(*cli_parser, argc, argv);
  }

  //
  const std::unique_ptr mem_resource = std::make_unique<zisc::AllocFreeResource>();

  {
    //
    const std::unique_ptr gltf_scene = std::make_unique<nanairo::GltfScene>(mem_resource.get());
    loadGltfScene(options.gltf_scene_path_, gltf_scene.get());

    //
    const std::unique_ptr renderer = std::make_unique<nanairo::Renderer>(mem_resource.get());
    renderer->initialize(options, *gltf_scene);

    // Create an LDR image for output
    const std::unique_ptr ldr_image = std::make_unique<nanairo::LdrImage>(mem_resource.get());
    ldr_image->initialize(options.image_width_, options.image_height_);

    //
    const std::unique_ptr png_writer = std::make_unique<nanairo::PngWriter>(mem_resource.get());

    for (std::size_t frame = options.min_frame_; frame < options.max_frame_; ++frame) {
      renderer->clearFrame();
      renderer->renderFrame(frame, 0);
      renderer->getFrame(ldr_image.get());
      png_writer->initialize();
      saveImage(frame, *ldr_image, *png_writer);
    }
  }

  return EXIT_SUCCESS;
}
