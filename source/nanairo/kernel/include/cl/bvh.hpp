/*!
  \file bvh.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_BVH_HPP
#define NANAIRO_CL_BVH_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "bvh_node_stack.hpp"
#include "hit_info.hpp"
#include "ray.hpp"

namespace nanairo {

//! Perform a ray traversal
HitInfo castRay(zivc::ConstGlobalPtr<zivc::uint32b> face_buffer,
                zivc::ConstGlobalPtr<float> geometry_buffer,
                zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                zivc::ConstGlobalPtr<zivc::uint32b> bvh_map_buffer,
                zivc::LocalPtr<BvhNodeStack> node_stack,
                const Ray ray) noexcept;

} /* namespace nanairo */

#include "bvh-inl.hpp"

#endif /* NANAIRO_CL_BVH_HPP */
