/*!
  \file cli.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "cli.hpp"
// Standard C++ library
#include <memory>
#include <string>
#include <string_view>

namespace nanairo {

/*!
  \details No detailed description

  \param [out] options No description.
  \return No description
  */
std::unique_ptr<CLI::App> createCommandLineParser(CliOptions* options) noexcept
{
  using namespace std::literals;

  constexpr std::string_view app_desc = "The path tracing renderer for RTC9."sv;
  constexpr std::string_view app_name = "Nanairo"sv;
  std::unique_ptr parser = std::make_unique<CLI::App>(app_desc.data(), app_name.data());

  {
    constexpr std::string_view desc = "Specify the file path to the gltf scene"sv;
    CLI::Option* option = parser->add_option("gltf", options->gltf_scene_path_, desc.data());
    option->required();
  }
  {
    constexpr std::string_view desc = "Specify the output image width."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_option("--width", options->image_width_, desc.data());
  }
  {
    constexpr std::string_view desc = "Specify the output image height."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_option("--height", options->image_height_, desc.data());
  }
  {
    constexpr std::string_view desc = "Specify the min frame of the animation render."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_option("--min-frame", options->min_frame_, desc.data());
  }
  {
    constexpr std::string_view desc = "Specify the max frame of the animation render."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_option("--max-frame", options->max_frame_, desc.data());
  }
  {
    constexpr std::string_view desc = "Specify the time budget (in milliseconds) for the animation rendering."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_option("--time-budget", options->time_budget_, desc.data());
  }
  {
    constexpr std::string_view desc = "Enable debug mode."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_flag("--debug", options->is_debug_mode_, desc.data());
  }
  {
    constexpr std::string_view desc = "Disable vulkan debug mode."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_flag("--no-vulkan-debug", options->exclude_vulkan_debug_mode_, desc.data());
  }
  {
    constexpr std::string_view desc = "Force using cpu as the kernel device."sv;
    [[maybe_unused]] CLI::Option* option = parser->add_flag("--force-cpu", options->is_cpu_forced_, desc.data());
  }

  return parser;
}

} /* namespace nanairo */
