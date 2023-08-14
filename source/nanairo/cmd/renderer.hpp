/*!
  \file renderer.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_RENDERER_HPP
#define NANAIRO_CMD_RENDERER_HPP

// Standard C++ library
#include <cstddef>
#include <memory>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"

namespace nanairo {

// Forward declaration
struct CliOptions;
class GltfScene;
class LdrImage;

/*!
  \brief No brief description

  No detailed description.
  */
class Renderer
{
 public:
  //! Create a renderer
  Renderer(zisc::pmr::memory_resource* mem_resource) noexcept;

  //! Destroy the renderer
  ~Renderer() noexcept;


  //! Destroy the renderer
  void destroy() noexcept;

  //! Initialize the renderer
  void initialize(const CliOptions& options, const GltfScene& scene);

  //! Render a frame
  void renderFrame(const std::size_t frame, LdrImage* output) noexcept;

 private:
  //! The kernel data for the renderer
  struct Data;


  zisc::pmr::memory_resource* mem_resource_ = nullptr;
  std::shared_ptr<Data> data_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_RENDERER_HPP */