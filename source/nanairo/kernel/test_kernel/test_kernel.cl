/*!
  \file test_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_KERNEL_TEST_KERNEL_CL
#define NANAIRO_KERNEL_TEST_KERNEL_CL

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/hit_info.hpp"
#include "cl/ray.hpp"

__kernel void testKernel(
    zivc::GlobalPtr<zivc::uint32b> ray_count,
    //zivc::ConstGlobalPtr<nanairo::Ray> ray,
    zivc::ConstGlobalPtr<nanairo::HitInfo> hit_info,
    zivc::GlobalPtr<float4> hdr_out)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  // test1
  //const nanairo::Ray r = ray[gindex];
  //const float3 value = (r.direction() + 1.0f) * 0.5f;

  // test2
  const nanairo::HitInfo info = hit_info[gindex];
  const float3 value = info.hasHit()
      ? (info.geometryNormal() + 1.0f) * 0.5f
      : zivc::makeFloat3(0.0f, 0.0f, 0.0f);
  //const float3 value = info.hasHit() ? zivc::makeFloat3(1.0f, 1.0f, 1.0f) : zivc::makeFloat3(0.0f, 0.0f, 0.0f);

  hdr_out[gindex].x = value.x;
  hdr_out[gindex].y = value.y;
  hdr_out[gindex].z = value.z;
}

#endif /* NANAIRO_KERNEL_TEST_KERNEL_CL */
