/*!
  \file gltf_scene.hpp
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

#ifndef NANAIRO_CMD_GLTF_SCENE_HPP
#define NANAIRO_CMD_GLTF_SCENE_HPP 1

// Standard C++ library
#include <istream>
#include <memory>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"

namespace cmd {

/*!
  \brief No brief description

  No detailed description.
  */
class GltfScene
{
 public:
  //! Create a gltf scene
  GltfScene(zisc::pmr::memory_resource* mem_resource) noexcept;

  //!
  ~GltfScene() noexcept;


  //! Destroy the scene
  void destroy() noexcept;

  //! Load a gltf scene
  void load(std::istream& data) noexcept;

 private:
};

} /* namespace cmd */

#endif /* NANAIRO_CMD_GLTF_SCENE_HPP */
