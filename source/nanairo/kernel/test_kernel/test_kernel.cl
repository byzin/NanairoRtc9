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
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/context_info.hpp"
#include "cl/primary_sample_set.hpp"

__kernel void testKernel(
    zivc::ConstGlobalPtr<nanairo::PrimarySampleSet> sample_set,
    zivc::GlobalPtr<float4> hdr_out,
    const nanairo::ContextInfo context_info)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint2 resolution = context_info.imageResolution();
  const uint32b n = resolution.x * resolution.y;
  const uint32b global_index = zivc::getGlobalIdX();
  if (n <= global_index)
    return;

  //
  const size_t set_offset = nanairo::PrimarySampleSet::getSetOffset(context_info.maxNumOfBounces());
}

#endif /* NANAIRO_KERNEL_TEST_KERNEL_CL */
