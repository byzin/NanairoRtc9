/*!
  \file feature_ray_cast_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_FEATURE_RAY_CAST_KERNEL_CL
#define NANAIRO_CL_FEATURE_RAY_CAST_KERNEL_CL

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/geometric.hpp"
#include "zivc/cl/math.hpp"
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

__kernel void castFeatureRayKernel(zivc::GlobalPtr<zivc::uint32b> ray_count,
                                   zivc::ConstGlobalPtr<nanairo::HitInfo> first_hit_info,
                                   zivc::ConstGlobalPtr<nanairo::Ray> feature_ray,
                                   zivc::ConstGlobalPtr<zivc::uint32b> face_buffer,
                                   zivc::ConstGlobalPtr<float> geometry_buffer,
                                   zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                                   zivc::ConstGlobalPtr<zivc::uint32b> bvh_map_buffer,
                                   zivc::GlobalPtr<zivc::uint8b> feature_line_count_buffer,
                                   zivc::GlobalPtr<nanairo::BvhNodeStack> bvh_node_stack_buffer,
                                   zivc::LocalPtr<nanairo::BvhNodeStack> bvh_node_stack)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  const nanairo::HitInfo first_hit = first_hit_info[gindex];
  if (!first_hit.hasHit())
    return;
  const zivc::uint8b line_count = feature_line_count_buffer[gindex];
  if (0 < line_count) {
    return;
  }

  const size_t lindex = zivc::getLocalIdX();
  zivc::LocalPtr<nanairo::BvhNodeStack> node_stack = bvh_node_stack + lindex;
  zivc::GlobalPtr<nanairo::BvhNodeStack> node_stack_buffer = bvh_node_stack_buffer + 2 * gindex;

  const nanairo::Ray r = feature_ray[gindex];
  const nanairo::HitInfo hit_info = nanairo::castRay(face_buffer,
                                                     geometry_buffer,
                                                     bvh_node_buffer,
                                                     bvh_map_buffer,
                                                     node_stack_buffer,
                                                     node_stack,
                                                     r);

  bool has_line = false;
  // object
  {
    const uint32b id1 = first_hit.meshId();
    const uint32b id2 = hit_info.meshId();
    has_line = id1 != id2;
  }
  // normal
  if (!has_line && hit_info.hasHit()) {
    constexpr float threshold = 45.0f;
    const float3 ng1 = first_hit.geometryNormal();
    const float3 ng2 = hit_info.geometryNormal();
    has_line = zivc::dot(ng1, ng2) < zivc::cos(zivc::radians(threshold));
  }
  if (has_line) {
    ++feature_line_count_buffer[gindex];
  }
}

#endif /* NANAIRO_CL_FEATURE_RAY_CAST_KERNEL_CL */
