/*!
  \file mesh.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "mesh.hpp"
// Standard C++ library
#include <memory>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
Mesh::Mesh(zisc::pmr::memory_resource* mem_resource) noexcept :
    vertices_{decltype(vertices_)::allocator_type{mem_resource}},
    normals_{decltype(normals_)::allocator_type{mem_resource}},
    texcoords_{decltype(texcoords_)::allocator_type{mem_resource}}
{
}

} /* namespace nanairo */
