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
#include <cstddef>
#include <memory>
#include <string>
// CLI11
#include "CLI/CLI.hpp"

namespace cmd {

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
  std::size_t max_frame_ = 100;
};

//! Create a command line parser
std::unique_ptr<CLI::App> createCommandLineParser(CliOptions* options) noexcept;

} /* namespace cmd */

#endif /* NANAIRO_CMD_CLI_HPP */
