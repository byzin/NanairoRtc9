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
#include <concepts>
#include <cstdint>
#include <istream>
#include <memory>
#include <span>
#include <vector>
// Tinygltf
#include "tiny_gltf.h"
// Zisc
#include "zisc/function_reference.hpp"
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
#include "camera.hpp"
#include "mesh.hpp"

namespace zivc::cl::nanairo {

// Forward declaration
struct Matrix4x4;

} /* namespace zivc::cl::nanairo */

namespace nanairo {

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


  //! Return the primary camera in the scene
  const Camera& camera() const noexcept;

  //! Destroy the scene
  void destroy() noexcept;

  //! Load a gltf scene
  void loadBinary(std::istream& data) noexcept;

  //! Return the list of the meshes in the scene
  std::span<const Mesh> meshList() const noexcept;

  //! Return the underlying model
  const tinygltf::Model& model() const noexcept;

 private:
  //
  using IndexGetterFuncT = zisc::FunctionReference<std::uint32_t (const unsigned char* const)>;
  using Matrix4x4 = zivc::cl::nanairo::Matrix4x4;


  //! Process geometries for rendering
  void compileGeometries() noexcept;

  //! Process a scene for rendering
  void compileScene() noexcept;

  //! Return an index getter corresponding to the given component type
  static IndexGetterFuncT getIndexGetter(const int component_type) noexcept;

  //!
  template <std::integral Integer>
  static std::uint32_t getIndexImpl(const unsigned char* const ptr) noexcept;

  //!
  zisc::pmr::memory_resource* resource() const noexcept;

  //!
  void printDebugInfo() const noexcept;

  //!
  void processCamera(const std::size_t index,
                     const Matrix4x4& transformation) noexcept;

  //!
  void processMesh(const std::size_t index,
                   const Matrix4x4& transformation,
                   const Matrix4x4& inv_transformation) noexcept;

  //!
  void processNode(const std::size_t index,
                   const Matrix4x4& parent_transformation,
                   const Matrix4x4& parent_inv_transformation,
                   const std::size_t level = 0) noexcept;


  Camera camera_;
  zisc::pmr::vector<Mesh> meshes_;
  tinygltf::Model model_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_GLTF_SCENE_HPP */
