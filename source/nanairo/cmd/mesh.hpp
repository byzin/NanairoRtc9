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
#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
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


  //! Write the mesh data as wavefront format
  void writeWavefrontFormat(std::ostream* output) const noexcept;


  zisc::pmr::vector<U3> faces_;
  zisc::pmr::vector<F3> vertices_;
  zisc::pmr::vector<F3> normals_;
  zisc::pmr::vector<F2> texcoords_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_MESH_HPP */
