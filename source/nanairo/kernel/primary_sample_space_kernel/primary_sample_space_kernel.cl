/*!
  \file primary_sample_space_kernel.cl
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL
#define NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL

// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"

using zivc::int32b;

__kernel void sampleInPrimarySampleSpaceKernel(zivc::GlobalPtr<int32b>)
{
}

#endif /* NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL */
