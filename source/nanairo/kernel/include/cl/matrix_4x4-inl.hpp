/*!
  \file matrix_4x4-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_MATRIX_4x4_INL_HPP
#define NANAIRO_CL_MATRIX_4x4_INL_HPP

#include "matrix_4x4.hpp"
// Zivc
#include "zivc/cl/geometric.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
Matrix4x4 Matrix4x4::identity() noexcept
{
  const float4 m1 = zivc::makeFloat4(1.0f, 0.0f, 0.0f, 0.0f);
  const float4 m2 = zivc::makeFloat4(0.0f, 1.0f, 0.0f, 0.0f);
  const float4 m3 = zivc::makeFloat4(0.0f, 0.0f, 1.0f, 0.0f);
  const float4 m4 = zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f);
  return {m1, m2, m3, m4};
}

/*!
  \details No detailed description

  \param [in] m1 No description.
  \param [in] m2 No description.
  \param [in] m3 No description.
  \param [in] m4 No description.
  */
inline
void Matrix4x4::set(const float4 m1, const float4 m2, const float4 m3, const float4 m4) noexcept
{
  m1_ = m1;
  m2_ = m2;
  m3_ = m3;
  m4_ = m4;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Matrix4x4 Matrix4x4::transpose() const noexcept
{
  const float4 m1 = zivc::makeFloat4(m1_.x, m2_.x, m3_.x, m4_.x);
  const float4 m2 = zivc::makeFloat4(m1_.y, m2_.y, m3_.y, m4_.y);
  const float4 m3 = zivc::makeFloat4(m1_.z, m2_.z, m3_.z, m4_.z);
  const float4 m4 = zivc::makeFloat4(m1_.w, m2_.w, m3_.w, m4_.w);
  return {m1, m2, m3, m4};
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
Matrix4x4 operator*(const Matrix4x4 lhs, const Matrix4x4 rhs) noexcept
{
  const Matrix4x4 r = rhs.transpose();
  const float4 m1 = zivc::makeFloat4(zivc::dot(lhs.m1_, r.m1_),
                                     zivc::dot(lhs.m1_, r.m2_),
                                     zivc::dot(lhs.m1_, r.m3_),
                                     zivc::dot(lhs.m1_, r.m4_));
  const float4 m2 = zivc::makeFloat4(zivc::dot(lhs.m2_, r.m1_),
                                     zivc::dot(lhs.m2_, r.m2_),
                                     zivc::dot(lhs.m2_, r.m3_),
                                     zivc::dot(lhs.m2_, r.m4_));
  const float4 m3 = zivc::makeFloat4(zivc::dot(lhs.m3_, r.m1_),
                                     zivc::dot(lhs.m3_, r.m2_),
                                     zivc::dot(lhs.m3_, r.m3_),
                                     zivc::dot(lhs.m3_, r.m4_));
  const float4 m4 = zivc::makeFloat4(zivc::dot(lhs.m4_, r.m1_),
                                     zivc::dot(lhs.m4_, r.m2_),
                                     zivc::dot(lhs.m4_, r.m3_),
                                     zivc::dot(lhs.m4_, r.m4_));
  return {m1, m2, m3, m4};
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
float4 operator*(const Matrix4x4 lhs, const float4 rhs) noexcept
{
  const float4 result = zivc::makeFloat4(zivc::dot(lhs.m1_, rhs),
                                         zivc::dot(lhs.m2_, rhs),
                                         zivc::dot(lhs.m3_, rhs),
                                         zivc::dot(lhs.m4_, rhs));
  return result;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_MATRIX_4x4_INL_HPP */
