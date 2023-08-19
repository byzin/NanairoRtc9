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
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "cl/context_info.hpp"
#include "cl/correlated_multi_jittered_engine.hpp"
#include "cl/pcg_hash_engine.hpp"
#include "cl/primary_sample_set.hpp"
#include "cl/render_info.hpp"

__kernel void sampleInPrimarySampleSpaceKernel(
    zivc::GlobalPtr<zivc::uint32b> ray_count,
    zivc::GlobalPtr<nanairo::PrimarySampleSet> sample_set,
    const nanairo::ContextInfo context_info,
    const nanairo::RenderInfo render_info)
{
  using zivc::int32b;
  using zivc::uint32b;
  using RngEngine = nanairo::CmjEngine4x4;
  using HashEngine = nanairo::PcgHashEngine;

  const uint32b gindex = zivc::getGlobalIdX();
  const uint32b n = zivc::atomic_load(ray_count);
  if (n <= gindex)
    return;

  const uint2 resolution = context_info.imageResolution();
  const uint32b num_of_pixels = resolution.x * resolution.y;

  // 
  const uint32b ite_offset = render_info.currentIteration() / RngEngine::getPeriod();
  const uint32b ite = render_info.currentIteration() % RngEngine::getPeriod();
  uint32b seed = gindex + ite_offset * num_of_pixels + context_info.seed();

  //
  const size_t set_size = nanairo::calcSampleSetSize(context_info.maxNumOfBounces());
  for (size_t i = 0; i < set_size; ++i) {
    nanairo::PrimarySampleSet samples{};
    for (size_t j = 0; j < nanairo::PrimarySampleSet::capacity(); j += 2) {
      seed = HashEngine::hash1D(seed);
      const float2 x = RngEngine::draw2D(ite, seed);
      samples.set(x.x, j + 0);
      samples.set(x.y, j + 1);
    }
    nanairo::setSampleSet(sample_set, samples, gindex, n, i);
  }
}

#endif /* NANAIRO_KERNEL_PRIMARY_SAMPLE_SPACE_KERNEL_CL */
