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
#include <chrono>
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
#include "zisc/stopwatch.hpp"
// Zivc
#include "zivc/zivc.hpp"
// Nanairo Cmd
#include "cli.hpp"
#include "gltf_scene.hpp"
#include "ldr_image.hpp"
#include "png_writer.hpp"
#include "renderer.hpp"

//
using ClockT = zisc::Stopwatch::Clock;

void fixOptionValue(nanairo::CliOptions* options) noexcept
{
  constexpr std::size_t min_resolution = 4;

  if (options->image_width_ < min_resolution)
    options->image_width_ = min_resolution;

  if (options->image_height_ < min_resolution)
    options->image_height_ = min_resolution;

  if (options->max_frame_ <= options->min_frame_)
    options->max_frame_ = options->min_frame_ + 1;
}

void printElapsedTime(const ClockT::duration& elapsed_time, const std::string_view label) noexcept
{
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed_time);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time - seconds);
  std::cout << "    " << label << ": "
                      << std::setw(6) << std::setfill('0') << seconds.count() << "s "
                      << std::setw(3) << std::setfill('0') << milliseconds.count() << "m"
                      << std::endl;
}

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
  std::snprintf(file_name.data(), file_name.size(), "%03d.png", static_cast<int>(frame));
  std::ofstream image_file{file_name.data(), std::ios_base::binary};
  writer.write(image, &image_file);
}

int main(const int argc, const char* const* const argv)
{
  zisc::Stopwatch stopwatch{};
  stopwatch.start();

  // Process command line arguments
  nanairo::CliOptions options{};
  {
    std::unique_ptr cli_parser = nanairo::createCommandLineParser(&options);
    CLI11_PARSE(*cli_parser, argc, argv);
  }
  fixOptionValue(&options);

  //
  const std::unique_ptr mem_resource = std::make_unique<zisc::AllocFreeResource>();
  const std::chrono::milliseconds time_budget{options.time_budget_};
  ClockT::duration elapsed_time{0};

  //
  auto check_time = [&stopwatch, &time_budget, &elapsed_time](const std::string_view label) noexcept
  {
    elapsed_time = stopwatch.elapsedTime();
    const bool flag = time_budget <= elapsed_time;
    if (flag) {
      std::cerr << "[error] The time is up before starting rendering." << std::endl;
      printElapsedTime(elapsed_time, label);
    }
    return flag;
  };
  if (check_time("Nanairo start"))
    return EXIT_FAILURE;

  {
    //
    const std::unique_ptr gltf_scene = std::make_unique<nanairo::GltfScene>(mem_resource.get());
    loadGltfScene(options.gltf_scene_path_, gltf_scene.get());
    if (check_time("Scene compilation"))
      return EXIT_FAILURE;

    //
    const std::unique_ptr renderer = std::make_unique<nanairo::Renderer>(mem_resource.get());
    renderer->initialize(options, *gltf_scene);
    if (check_time("Render device initialization"))
      return EXIT_FAILURE;

    // Create an LDR image for output
    const std::unique_ptr ldr_image = std::make_unique<nanairo::LdrImage>(mem_resource.get());
    const std::unique_ptr png_writer = std::make_unique<nanairo::PngWriter>(mem_resource.get());
    ldr_image->initialize(options.image_width_, options.image_height_);
    if (check_time("Output initialization"))
      return EXIT_FAILURE;
    if (elapsed_time = stopwatch.elapsedTime(); options.is_debug_mode_)
      printElapsedTime(elapsed_time, "Rendering start");

    // Calculate the time budget per frame
    const std::size_t num_of_frames = options.max_frame_ - options.min_frame_;
    const auto time_budget_per_frame = std::chrono::duration_cast<std::chrono::nanoseconds>(time_budget - elapsed_time) / num_of_frames;
    auto check_frame_time = [&stopwatch, &time_budget_per_frame, &elapsed_time](const std::size_t iteration)
    {
      const std::size_t n = iteration + 1;
      const std::chrono::nanoseconds sliced_budget = time_budget_per_frame / n;
      const ClockT::duration estimated_time_it = (stopwatch.elapsedTime() - elapsed_time) / n;
      return (n == 1) || (estimated_time_it < sliced_budget);
    };

    //
    for (std::size_t frame = options.min_frame_; frame < options.max_frame_; ++frame) {
      renderer->clearFrame();
      for (std::size_t iteration = 0; check_frame_time(iteration); ++iteration)
        renderer->renderFrame(frame, iteration);
      renderer->getFrame(ldr_image.get());
      png_writer->initialize();
      saveImage(frame, *ldr_image, *png_writer);
      if (check_time("Render"))
        break;
    }

    if (elapsed_time = stopwatch.elapsedTime(); options.is_debug_mode_)
      printElapsedTime(elapsed_time, "Rendering end");
  }

  return EXIT_SUCCESS;
}
