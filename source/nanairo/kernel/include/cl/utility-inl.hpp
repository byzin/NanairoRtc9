/*!
  \file utility-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_UTILITY_INL_HPP
#define NANAIRO_CL_UTILITY_INL_HPP

#include "utility.hpp"
// Zivc
#include "zivc/cl/limits.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] value No description.
  \return No description
  */
inline
uint16b Utility::getLowerU16(const uint32b value) noexcept
{
  const auto v = static_cast<uint16b>(value & u16Mask());
  return v;
}

/*!
  \details No detailed description

  \param [in] value No description.
  \return No description
  */
inline
uint16b Utility::getUpperU16(const uint32b value) noexcept
{
  using LimitT = zivc::NumericLimits<uint16b>;
  const auto v = static_cast<uint16b>(value >> LimitT::digits());
  return v;
}

/*!
  \details Please read "Generating uniform doubles in the unit interval" section
  in the following link:
  <a href="http://prng.di.unimi.it/">xoshiro / xoroshiro generators and the PRNG shootout</a> for more details.

  \param [in] x No description.
  \return No description
  */
inline
float Utility::mapTo01(const uint32b x) noexcept
{
  constexpr size_t int_digits = zivc::NumericLimits<uint32b>::digits();
  constexpr size_t f_mant_digits = zivc::NumericLimits<float>::digits();
  constexpr size_t offset = int_digits - f_mant_digits;
  constexpr float norm = 1.0f / static_cast<float>(1u << f_mant_digits);
  const float y = static_cast<float>(x >> offset) * norm;
  return y;
}

/*!
  \details Please read "Generating uniform doubles in the unit interval" section
  in the following link:
  <a href="http://prng.di.unimi.it/">xoshiro / xoroshiro generators and the PRNG shootout</a> for more details.

  \param [in] x No description.
  \return No description
  */
inline
float2 Utility::mapTo01(const uint2 x) noexcept
{
  constexpr size_t int_digits = zivc::NumericLimits<uint32b>::digits();
  constexpr size_t f_mant_digits = zivc::NumericLimits<float>::digits();
  constexpr size_t offset = int_digits - f_mant_digits;
  constexpr float norm = 1.0f / static_cast<float>(1u << f_mant_digits);
  const float2 y = zivc::Utility::cast<float2>(x >> offset) * norm;
  return y;
}

/*!
  \details No detailed description

  \param [in] x No description.
  \return No description
  */
inline
uint32b Utility::mapToInt(const float x) noexcept
{
  constexpr size_t int_digits = zivc::NumericLimits<uint32b>::digits();
  constexpr size_t f_mant_digits = zivc::NumericLimits<float>::digits();
  constexpr size_t offset = int_digits - f_mant_digits;
  constexpr auto scale = static_cast<float>(1u << f_mant_digits);
  const uint32b y = static_cast<uint32b>(x * scale) << offset;
  return y;
}

/*!
  \details No detailed description

  \param [in] x No description.
  \return No description
  */
inline
uint2 Utility::mapToInt(const float2 x) noexcept
{
  constexpr size_t int_digits = zivc::NumericLimits<uint32b>::digits();
  constexpr size_t f_mant_digits = zivc::NumericLimits<float>::digits();
  constexpr size_t offset = int_digits - f_mant_digits;
  constexpr auto scale = static_cast<float>(1u << f_mant_digits);
  const uint2 y = zivc::Utility::cast<uint2>(x * scale) << offset;
  return y;
}

/*!
  \details No detailed description

  \param [in] value No description.
  \param [in,out] dest No description.
  */
inline
uint32b Utility::setLowerU16(const uint16b value, const uint32b dest) noexcept
{
  using LimitT = zivc::NumericLimits<uint16b>;
  const uint32b u = static_cast<uint32b>(getUpperU16(dest)) << LimitT::digits();
  const uint32b l = static_cast<uint32b>(value);
  const uint32b result = u | l;
  return result;
}

/*!
  \details No detailed description

  \param [in] value No description.
  \param [in,out] dest No description.
  */
inline
uint32b Utility::setUpperU16(const uint16b value, const uint32b dest) noexcept
{
  using LimitT = zivc::NumericLimits<uint16b>;
  const uint32b u = static_cast<uint32b>(value) << LimitT::digits();
  const uint32b l = static_cast<uint32b>(getLowerU16(dest));
  const uint32b result = u | l;
  return result;
}

/*!
  \details No detailed description

  \return No description
  */
inline
constexpr uint32b Utility::u16Mask() noexcept
{
  using LimitT = zivc::NumericLimits<uint16b>;
  const uint32b mask = (0b1u << LimitT::digits()) - 1u;
  return mask;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_UTILITY_INL_HPP */
