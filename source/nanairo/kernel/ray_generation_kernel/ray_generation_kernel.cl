/*!
  \file ray_generation_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_KERNEL_RAY_GENERATION_KERNEL_CL
#define NANAIRO_KERNEL_RAY_GENERATION_KERNEL_CL

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/camera.hpp"
#include "cl/camera_info.hpp"
#include "cl/context_info.hpp"
#include "cl/primary_sample_set.hpp"
#include "cl/ray.hpp"

__kernel void generateRayKernel(
    zivc::GlobalPtr<zivc::uint32b> ray_count,
    zivc::ConstGlobalPtr<nanairo::PrimarySampleSet> sample_set,
    zivc::GlobalPtr<nanairo::Ray> ray,
    const nanairo::ContextInfo context_info,
    const nanairo::CameraInfo camera_info)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  //
  const float2 samples = nanairo::getSample2D(sample_set,
                                              gindex,
                                              n,
                                              nanairo::SampleSetUsage::kImagePlaneX,
                                              0);
  const uint2 resolution = context_info.imageResolution();
  const nanairo::Ray r = nanairo::generateRay(camera_info, resolution, gindex, samples);
  ray[gindex] = r;
}

#endif /* NANAIRO_KERNEL_RAY_GENERATION_KERNEL_CL */
