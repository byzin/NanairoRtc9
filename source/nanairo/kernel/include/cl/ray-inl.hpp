/*!
  \file ray-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_RAY_INL_HPP
#define NANAIRO_CL_RAY_INL_HPP

#include "ray.hpp"
// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
float3 Ray::direction() const noexcept
{
  return zivc::makeFloat3(dx_, dy_, dz_);
}

/*!
  \details No detailed description

  \return No description
  */
inline
float3 Ray::origin() const noexcept
{
  return zivc::makeFloat3(ox_, oy_, oz_);
}

/*!
  \details No detailed description

  \param [in] dir No description.
  */
inline
void Ray::setDirection(const float3 dir) noexcept
{
  dx_ = dir.x;
  dy_ = dir.y;
  dz_ = dir.z;
}

/*!
  \details No detailed description

  \param [in] origin No description.
  */
inline
void Ray::setOrigin(const float3 origin) noexcept
{
  ox_ = origin.x;
  oy_ = origin.y;
  oz_ = origin.z;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_RAY_INL_HPP */
