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
#include <ostream>
#include <vector>
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

/*!
  \details No detailed description

  \param [out] output No description.
  */
void Mesh::writeWavefrontFormat(std::ostream* output) const noexcept
{
  (*output) << "o Mesh" << std::endl;
  for (const F3& vertex : vertices_)
    (*output) << "v " << vertex.x_ << " " << vertex.y_ << " " << vertex.z_ << std::endl;
  for (const F3& normal : normals_)
    (*output) << "vn " << normal.x_ << " " << normal.y_ << " " << normal.z_ << std::endl;
  for (const F2& texcoord : texcoords_)
    (*output) << "vt " << texcoord.x_ << " " << texcoord.y_ << std::endl;
  (*output) << "s 0" << std::endl;
  for (const U3& face : faces_)
    //(*output) << "f " << face.x_ << "/" << face.x_ << "/" << face.x_ << " "
    //                  << face.y_ << "/" << face.y_ << "/" << face.y_ << " "
    //                  << face.z_ << "/" << face.z_ << "/" << face.z_ << std::endl;
    (*output) << "f " << (face.x_ + 1) << " "
                      << (face.y_ + 1) << " "
                      << (face.z_ + 1) << std::endl;
}

} /* namespace nanairo */
