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

#ifndef NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL_CL
#define NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL_CL

// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/context_info.hpp"
#include "cl/correlated_multi_jittered_engine.hpp"
#include "cl/pcg_hash_engine.hpp"
#include "cl/primary_sample_set.hpp"
#include "cl/render_info.hpp"

__kernel void sampleInPrimarySampleSpaceKernel(
    zivc::GlobalPtr<nanairo::PrimarySampleSet> sample_set,
    const nanairo::ContextInfo context_info,
    const nanairo::RenderInfo render_info)
{
  using zivc::int32b;
  using zivc::uint32b;
  using RngEngine = nanairo::CmjEngine4x4;
  using HashEngine = nanairo::PcgHashEngine;

  const uint2 resolution = context_info.imageResolution();
  const uint32b n = resolution.x * resolution.y;
  const uint32b global_index = zivc::getGlobalIdX();
  if (n <= global_index)
    return;

  // 
  const uint32b ite_offset = render_info.currentIteration() / RngEngine::getPeriod();
  const uint32b ite = render_info.currentIteration() % RngEngine::getPeriod();
  uint32b seed = global_index + ite_offset * n + context_info.seed();

  //
  const size_t set_offset = nanairo::PrimarySampleSet::getSetOffset(context_info.maxNumOfBounces());
  zivc::GlobalPtr<nanairo::PrimarySampleSet> set = sample_set + set_offset * global_index;
  for (size_t i = 0; i < set_offset; ++i) {
    nanairo::PrimarySampleSet samples{};
    for (size_t j = 0; j < nanairo::PrimarySampleSet::capacity(); j += 2) {
      seed = HashEngine::hash1D(seed);
      const float2 x = RngEngine::draw2D(ite, seed);
      samples.set(x.x, j + 0);
      samples.set(x.y, j + 1);
    }
    set[i] = samples;
  }
}

#endif /* NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL_CL */
