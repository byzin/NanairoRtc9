/*!
  \file quaternion-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_QUATERNION_INL_HPP
#define NANAIRO_CL_QUATERNION_INL_HPP

#include "quaternion.hpp"
// Zivc
#include "zivc/cl/bit.hpp"
#include "zivc/cl/geometric.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
float4 Quaternion::data() const noexcept
{
  return data_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Quaternion Quaternion::getConjugate() const noexcept
{
  const Quaternion result = {zivc::makeFloat4(-data_.x, -data_.y, -data_.z, data_.w)};
  return result;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Quaternion Quaternion::identity() noexcept
{
  const Quaternion i = {zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f)};
  return i;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Quaternion Quaternion::invert() const noexcept
{
  // Assume this is a rotation quaternion so the norm is 1
  return getConjugate();
}

/*!
  \details No detailed description

  \param [in] vec No description.
  \return No description
  */
inline
float3 Quaternion::rotate(const float3 vec) const noexcept
{
  const Quaternion v = {zivc::makeFloat4(vec.x, vec.y, vec.z, 0.0f)};
  const Quaternion result = (*this) * v * invert();
  return zivc::castBit<float3>(result.data());
}

/*!
  \details No detailed description

  \param [in] x No description.
  \param [in] y No description.
  \param [in] z No description.
  \param [in] angle No description.
  */
inline
void Quaternion::set(const float x, const float y, const float z, const float angle) noexcept
{
  set(zivc::makeFloat4(x, y, z, angle));
}

/*!
  \details No detailed description

  \param [in] data No description.
  */
inline
void Quaternion::set(const float4 data) noexcept
{
  data_ = data;
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
Quaternion operator*(const Quaternion lhs, const Quaternion rhs) noexcept
{
  const float4 q = lhs.data();
  const float4 p = rhs.data();

  const float3 q3 = zivc::castBit<float3>(q);
  const float3 p3 = zivc::castBit<float3>(p);
  const float3 r3 = zivc::cross(q3, p3) + q.w * p3 + p.w * q3;

  float4 r = zivc::castBit<float4>(r3);
  r.w = q.w * p.w - zivc::dot(q3, p3);

  const Quaternion result = {r};
  return result;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_QUATERNION_INL_HPP */
