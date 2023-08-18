/*!
  \file context_info-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CONTEXT_INFO_INL_HPP
#define NANAIRO_CL_CONTEXT_INFO_INL_HPP

#include "context_info.hpp"
// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"
#include "utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
uint2 ContextInfo::imageResolution() const noexcept
{
  const uint2 resolution = zivc::makeUInt2(Utility::getLowerU16(data0_.x),
                                           Utility::getUpperU16(data0_.x));
  return resolution;
}

/*!
  \details No detailed description
  */
inline
void ContextInfo::initialize() noexcept
{
  data0_ = zivc::makeUInt4(0u);
}

/*!
  \details No detailed description

  \return No description
  */
inline
uint16b ContextInfo::maxNumOfBounces() const noexcept
{
  return Utility::getUpperU16(data0_.z);
}

/*!
  \details No detailed description

  \return No description
  */
inline
uint32b ContextInfo::seed() const noexcept
{
  return data0_.y;
}

/*!
  \details No detailed description

  \param [in] resolution No description.
  */
inline
void ContextInfo::setImageResolution(const ushort2 resolution) noexcept
{
  data0_.x = Utility::setLowerU16(resolution.x, data0_.x);
  data0_.x = Utility::setUpperU16(resolution.y, data0_.x);
}

/*!
  \details No detailed description

  \param [in] max_bounces No description.
  */
inline
void ContextInfo::setMaxNumOfBounces(const uint16b max_bounces) noexcept
{
  data0_.z = Utility::setUpperU16(max_bounces, data0_.z);
}

/*!
  \details No detailed description

  \param [in] seed No description.
  */
inline
void ContextInfo::setSeed(const uint32b seed) noexcept
{
  data0_.y = seed;
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
bool operator==(const ContextInfo& lhs, const ContextInfo& rhs) noexcept
{
  const bool result = (lhs.data0_.x == rhs.data0_.x) &&
                      (lhs.data0_.y == rhs.data0_.y) &&
                      (lhs.data0_.z == rhs.data0_.z) &&
                      (lhs.data0_.w == rhs.data0_.w);
  return result;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_CONTEXT_INFO_INL_HPP */
