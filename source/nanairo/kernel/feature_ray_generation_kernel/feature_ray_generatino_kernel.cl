/*!
  \file feature_ray_generation_kernel\cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_KERNEL_FEATURE_RAY_GENERATION_KERNEL_CL
#define NANAIRO_KERNEL_FEATURE_RAY_GENERATION_KERNEL_CL

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/math.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/camera.hpp"
#include "cl/camera_info.hpp"
#include "cl/context_info.hpp"
#include "cl/feature_ray.hpp"
#include "cl/hit_info.hpp"
#include "cl/primary_sample_set.hpp"
#include "cl/ray.hpp"

__kernel void generateFeatureRayKernel(zivc::GlobalPtr<zivc::uint32b> ray_count,
                                       zivc::ConstGlobalPtr<nanairo::PrimarySampleSet> sample_set,
                                       zivc::ConstGlobalPtr<nanairo::Ray> ray,
                                       zivc::ConstGlobalPtr<nanairo::HitInfo> hit_info,
                                       zivc::GlobalPtr<nanairo::Ray> feature_ray,
                                       const nanairo::ContextInfo context_info,
                                       const nanairo::CameraInfo camera_info,
                                       const zivc::uint32b count)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  nanairo::generateFeatureRay(sample_set,
                              ray,
                              hit_info,
                              feature_ray,
                              gindex,
                              n,
                              context_info,
                              camera_info,
                              count);
}

#endif /* NANAIRO_KERNEL_FEATURE_RAY_GENERATION_KERNEL_CL */
