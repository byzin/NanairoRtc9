/*!
  \file pcg_hash_engine-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_PCG_HASH_ENGINE_INL_HPP
#define NANAIRO_CL_PCG_HASH_ENGINE_INL_HPP

#include "pcg_hash_engine.hpp"
// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] x No description.
  \return No description
  */
inline
uint32b PcgHashEngine::hash1D(const uint32b x) noexcept
{
  const uint32b state = x * 747796405u + 2891336453u;
  const uint32b word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
  const uint32b y = (word >> 22u) ^ word;
  return y;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_PCG_HASH_ENGINE_INL_HPP */
