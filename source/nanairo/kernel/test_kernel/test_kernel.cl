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
#include "cl/primary_sample_set.hpp"

__kernel void testKernel(
    zivc::GlobalPtr<zivc::uint32b> ray_count,
    zivc::ConstGlobalPtr<nanairo::PrimarySampleSet> sample_set,
    zivc::GlobalPtr<float4> hdr_out)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  //
  const float2 sample = nanairo::getSample2D(sample_set,
                                             gindex,
                                             n,
                                             //nanairo::SampleSetUsage::kImagePlaneX,
                                             //nanairo::SampleSetUsage::kBsdfSampleX,
                                             nanairo::SampleSetUsage::kLightSampleX,
                                             1);
  hdr_out[gindex].x = sample.x;
  hdr_out[gindex].y = sample.y;
}

#endif /* NANAIRO_KERNEL_TEST_KERNEL_CL */
