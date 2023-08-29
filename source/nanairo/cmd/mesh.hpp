/*!
  \file mesh.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_MESH_HPP
#define NANAIRO_CMD_MESH_HPP

// Standard C++ library
#include <array>
#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
#include "bvh_node.hpp"
#include "utility.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
class Mesh
{
 public:
  //! Create a mesh
  Mesh(zisc::pmr::memory_resource* mem_resource) noexcept;


  //! Build the bottom-level acceleration structure
  void compileBlas() noexcept;

  //! Return the resource of the mesh
  zisc::pmr::memory_resource* resource() const noexcept;

  //! Write the mesh data as wavefront format
  void writeWavefrontFormat(std::ostream* output) const noexcept;


  zisc::pmr::vector<U3> faces_;
  zisc::pmr::vector<F3> vertices_;
  zisc::pmr::vector<F3> normals_;
  zisc::pmr::vector<F2> texcoords_;
  std::array<float, 16> inv_transformation_;
  // BVH
  std::array<float, 6> aabb_;
  zisc::pmr::vector<std::array<float, 6>> face_aabb_;
  zisc::pmr::vector<std::uint32_t> face_code_;
  zisc::pmr::vector<BvhNode> bvh_node_;
  zisc::pmr::vector<std::size_t> bvh_leaf_node_;
  BvhInfo bvh_info_;

 private:
  //!
  void buildBvh() noexcept;

  //! Calculate the mesh AABB from the vertices
  void calcAabb() noexcept;

  //!
  void calcMortonCode() noexcept;

  //! Calculate the node AABB
  std::array<float, 6> calcNodeAabb(const std::size_t index, const std::size_t level) noexcept;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_MESH_HPP */
