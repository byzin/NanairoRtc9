/*!
  \file hit_info-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_HIT_INFO_INL_HPP
#define NANAIRO_CL_HIT_INFO_INL_HPP

#include "hit_info.hpp"
// Zivc
#include "zivc/cl/limits.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
float HitInfo::distance() const noexcept
{
  return distance_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
float3 HitInfo::geometryNormal() const noexcept
{
  return zivc::makeFloat3(ng_x_, ng_y_, ng_z_);
}

inline
zivc::uint32b HitInfo::meshId() const noexcept
{
  return mesh_id_;
}

inline
zivc::uint32b HitInfo::trisId() const noexcept
{
  return tris_id_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
bool HitInfo::hasHit() const noexcept
{
  using LimitT = zivc::NumericLimits<float>;
  const bool flag = distance() != LimitT::infinity();
  return flag;
}

/*!
  \details No detailed description
  */
inline
void HitInfo::initialize() noexcept
{
  using FLimitT = zivc::NumericLimits<float>;
  using ULimitT = zivc::NumericLimits<zivc::uint32b>;
  setDistance(FLimitT::infinity());
  setGeometryNormal(zivc::makeFloat3(0.0f, 0.0f, 0.0f));
  setMeshId(ULimitT::max());
  setTrisId(ULimitT::max());
}

/*!
  \details No detailed description

  \param [in] d No description.
  */
inline
void HitInfo::setDistance(const float d) noexcept
{
  distance_ = d;
}

/*!
  \details No detailed description

  \param [in] d No description.
  */
inline
void HitInfo::setGeometryNormal(const float3 n) noexcept
{
  ng_x_ = n.x;
  ng_y_ = n.y;
  ng_z_ = n.z;
}

inline
void HitInfo::setMeshId(const zivc::uint32b mesh_id) noexcept
{
  mesh_id_ = mesh_id;
}

inline
void HitInfo::setTrisId(const zivc::uint32b tris_id) noexcept
{
  tris_id_ = tris_id;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_HIT_INFO_INL_HPP */
