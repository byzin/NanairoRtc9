/*!
  \file utility.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_UTILITY_HPP
#define NANAIRO_CL_UTILITY_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct Utility 
{
  //! Return the lower 16 bits of the given u32
  static uint16b getLowerU16(const uint32b value) noexcept;

  //! Return the upper 16 bits of the given u32
  static uint16b getUpperU16(const uint32b value) noexcept;

  //! Map an integer value into a [0, 1) floating point value
  static float mapTo01(const uint32b x) noexcept;

  //! Map an integer value into a [0, 1) floating point value
  static float2 mapTo01(const uint2 x) noexcept;

  //! Map a [0, 1) floating point value into an integer value
  static uint32b mapToInt(const float x) noexcept;

  //! Map a [0, 1) floating point value into an integer value
  static uint2 mapToInt(const float2 x) noexcept;

  //! Set the given u16 value into the lower 16 bits of the dest u32
  static uint32b setLowerU16(const uint16b value, const uint32b dest) noexcept;

  //! Set the given u16 value into the upper 16 bits of the dest u32
  static uint32b setUpperU16(const uint16b value, const uint32b dest) noexcept;

  //! Return the mask of uint16
  static constexpr uint32b u16Mask() noexcept;
};

} /* namespace nanairo */

#include "utility-inl.hpp"

#endif /* NANAIRO_CL_UTILITY_HPP */
