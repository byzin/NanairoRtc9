/*!
  \file context_info.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CONTEXT_INFO_HPP
#define NANAIRO_CL_CONTEXT_INFO_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct ContextInfo
{
  //! Return the image resolution
  uint2 imageResolution() const noexcept;

  //! Initialize the context info
  void initialize() noexcept;

  //! Return the maximum possible number of bounces
  uint16b maxNumOfBounces() const noexcept;

  //! Return the seed for sampling
  uint32b seed() const noexcept;

  //! Set the image resolution
  void setImageResolution(const ushort2 resolution) noexcept;

  //! Set the maximum possible number of bounces
  void setMaxNumOfBounces(const uint16b max_bounces) noexcept;

  //! Set the seed for sampling
  void setSeed(const uint32b seed) noexcept;


  uint4 data0_;
};

//! Check if the two info have same values
bool operator==(const ContextInfo& lhs, const ContextInfo& rhs) noexcept;

} /* namespace nanairo */

#include "context_info-inl.hpp"

#endif /* NANAIRO_CL_CONTEXT_INFO_HPP */
