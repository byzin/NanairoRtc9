/*!
  \file render_info-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_RENDER_INFO_INL_HPP
#define NANAIRO_CL_RENDER_INFO_INL_HPP

#include "render_info.hpp"
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
uint16b RenderInfo::currentFrame() const noexcept
{
  return Utility::getLowerU16(data0_.x);
}

/*!
  \details No detailed description

  \return No description
  */
inline
uint16b RenderInfo::currentIteration() const noexcept
{
  return Utility::getUpperU16(data0_.x);
}

/*!
  \details No detailed description
  */
inline
void RenderInfo::initialize() noexcept
{
  data0_ = zivc::makeUInt4(0u);
}

/*!
  \details No detailed description

  \param [in] frame No description.
  */
inline
void RenderInfo::setCurrentFrame(const uint16b frame) noexcept
{
  data0_.x = Utility::setLowerU16(frame, data0_.x);
}

/*!
  \details No detailed description

  \param [in] iteration No description.
  */
inline
void RenderInfo::setCurrentIteration(const uint16b iteration) noexcept
{
  data0_.x = Utility::setUpperU16(iteration, data0_.x);
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
bool operator==(const RenderInfo& lhs, const RenderInfo& rhs) noexcept
{
  const bool result = (lhs.data0_.x == rhs.data0_.x) &&
                      (lhs.data0_.y == rhs.data0_.y) &&
                      (lhs.data0_.z == rhs.data0_.z) &&
                      (lhs.data0_.w == rhs.data0_.w);
  return result;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_RENDER_INFO_INL_HPP */
