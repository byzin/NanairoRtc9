/*!
  \file ray.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_RAY_HPP
#define NANAIRO_CL_RAY_HPP

// Zivc
#include "zivc/cl/types.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct Ray
{
  //! Return the direction of the ray
  float3 direction() const noexcept;

  //! Return the origin of the ray
  float3 origin() const noexcept;

  //! Set the direction of the ray
  void setDirection(const float3 dir) noexcept;

  //! Set the origin of the ray
  void setOrigin(const float3 origin) noexcept;


  float ox_,
        oy_,
        oz_;
  float dx_,
        dy_,
        dz_;
};

} /* namespace nanairo */

#include "ray-inl.hpp"

#endif /* NANAIRO_CL_RAY_HPP */
