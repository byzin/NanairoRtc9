/*!
  \file tone_mapping_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_KERNEL_TONE_MAPPING_KERNEL_CL
#define NANAIRO_KERNEL_TONE_MAPPING_KERNEL_CL

// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/context_info.hpp"

__kernel void applyToneMappingKernel(
    zivc::ConstGlobalPtr<float4> hdr_in,
    zivc::GlobalPtr<uchar4> ldr_out,
    const nanairo::ContextInfo context_info)
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint2 resolution = context_info.imageResolution();
  const uint32b n = resolution.x * resolution.y;
  const uint32b global_index = zivc::getGlobalIdX();
  if (n <= global_index)
    return;

  float4 value = hdr_in[global_index];
  value.w = 1.0f;
  value = zivc::clamp(255.5f * value, zivc::makeFloat4(0.0f), zivc::makeFloat4(255.0f));
  ldr_out[global_index] = zivc::cast<uchar4>(value);
}

#endif /* NANAIRO_KERNEL_TONE_MAPPING_KERNEL_CL */
