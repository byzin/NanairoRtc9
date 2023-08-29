/*!
  \file hit_info.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_HIT_INFO_HPP
#define NANAIRO_CL_HIT_INFO_HPP

// Zivc
#include "zivc/cl/types.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct HitInfo
{
  //! Return the hit distance
  float distance() const noexcept;

  //! Return the geometry normal
  float3 geometryNormal() const noexcept;

  //! Check if the hit has hit
  bool hasHit() const noexcept;

  //!
  void initialize() noexcept;

  //! Set the hit distance
  void setDistance(const float d) noexcept;

  //! Set the geometry normal
  void setGeometryNormal(const float3 n) noexcept;


  float distance_;
  float ng_x_,
        ng_y_,
        ng_z_;
};

} /* namespace nanairo */

#include "hit_info-inl.hpp"

#endif /* NANAIRO_CL_HIT_INFO_HPP */
