/*!
  \file cli.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \note No notation.
  \attention No attention.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_CLI_HPP
#define NANAIRO_CMD_CLI_HPP 1

// Standard C++ library
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
// CLI11
// Workaround errors with X11
#undef Success
#include "CLI/CLI.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct CliOptions
{
  std::string gltf_scene_path_;
  std::size_t image_width_ = 1920;
  std::size_t image_height_ = 1080;
  std::size_t min_frame_ = 0;
  std::size_t max_frame_ = 30;
  std::size_t time_budget_ = 295'000;
  bool is_debug_mode_ = false;
  bool exclude_vulkan_debug_mode_ = false;
  bool is_cpu_forced_ = false;
  [[maybe_unused]] std::array<std::uint8_t, 5> padd_;
};

static_assert(sizeof(bool) == 1);

//! Create a command line parser
std::unique_ptr<CLI::App> createCommandLineParser(CliOptions* options) noexcept;

} /* namespace nanairo */

#endif /* NANAIRO_CMD_CLI_HPP */
