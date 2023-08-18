/*!
  \file pcg_hash_engine.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_PCG_HASH_ENGINE_HPP
#define NANAIRO_CL_PCG_HASH_ENGINE_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
class PcgHashEngine
{
 public:
  //! Hash function for 1d
  static uint32b hash1D(const uint32b x) noexcept;
};

} /* namespace nanairo */

#include "pcg_hash_engine-inl.hpp"

#endif /* NANAIRO_CL_PCG_HASH_ENGINE_HPP */
