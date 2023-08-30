/*!
  \file ray_cast_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_RAY_CAST_KERNEL_CL
#define NANAIRO_CL_RAY_CAST_KERNEL_CL

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/bvh.hpp"
#include "cl/bvh_info.hpp"
#include "cl/bvh_node.hpp"
#include "cl/bvh_node_stack.hpp"
#include "cl/hit_info.hpp"
#include "cl/matrix_4x4.hpp"
#include "cl/ray.hpp"
#include "cl/transformation.hpp"

__kernel void castRayKernel(zivc::GlobalPtr<zivc::uint32b> ray_count,
                            zivc::ConstGlobalPtr<nanairo::Ray> ray,
                            zivc::ConstGlobalPtr<zivc::uint32b> face_buffer,
                            zivc::ConstGlobalPtr<float> geometry_buffer,
                            zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                            zivc::ConstGlobalPtr<zivc::uint32b> bvh_map_buffer,
                            zivc::GlobalPtr<nanairo::HitInfo> hit_info_buffer,
                            zivc::GlobalPtr<nanairo::BvhNodeStack> bvh_node_stack_buffer,
                            zivc::LocalPtr<nanairo::BvhNodeStack> bvh_node_stack)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  const size_t lindex = zivc::getLocalIdX();
  zivc::GlobalPtr<nanairo::BvhNodeStack> node_stack_buffer = bvh_node_stack_buffer + 2 * gindex;
  zivc::LocalPtr<nanairo::BvhNodeStack> node_stack = bvh_node_stack + lindex;

  const nanairo::Ray r = ray[gindex];
  const nanairo::HitInfo hit_info = nanairo::castRay(face_buffer,
                                                     geometry_buffer,
                                                     bvh_node_buffer,
                                                     bvh_map_buffer,
                                                     node_stack_buffer,
                                                     node_stack,
                                                     r);
  hit_info_buffer[gindex] = hit_info;
}

#endif /* NANAIRO_CL_RAY_CAST_KERNEL_CL */
