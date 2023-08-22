/*!
  \file transformation-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_TRANSFORMATION_INL_HPP
#define NANAIRO_CL_TRANSFORMATION_INL_HPP

#include "transformation.hpp"
// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "matrix_4x4.hpp"
#include "quaternion.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] tx No description.
  \param [in] ty No description.
  \param [in] tz No description.
  \return No description
  */
inline
Matrix4x4 getTranslationMatrix(const float tx, const float ty, const float tz) noexcept
{
  const Matrix4x4 m = {zivc::makeFloat4(1.0f, 0.0f, 0.0f, tx),
                       zivc::makeFloat4(0.0f, 1.0f, 0.0f, ty),
                       zivc::makeFloat4(0.0f, 0.0f, 1.0f, tz),
                       zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f)};
  return m;
}

/*!
  \details No detailed description

  \param [in] tx No description.
  \param [in] ty No description.
  \param [in] tz No description.
  \return No description
  */
inline
Matrix4x4 getInvTranslationMatrix(const float tx, const float ty, const float tz) noexcept
{
  const Matrix4x4 m = {zivc::makeFloat4(1.0f, 0.0f, 0.0f, -tx),
                       zivc::makeFloat4(0.0f, 1.0f, 0.0f, -ty),
                       zivc::makeFloat4(0.0f, 0.0f, 1.0f, -tz),
                       zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f)};
  return m;
}

/*!
  \details No detailed description

  \param [in] sx No description.
  \param [in] sy No description.
  \param [in] sz No description.
  \return No description
  */
inline
Matrix4x4 getScalingMatrix(const float sx, const float sy, const float sz) noexcept
{
  const Matrix4x4 m = {zivc::makeFloat4(sx, 0.0f, 0.0f, 0.0f),
                       zivc::makeFloat4(0.0f, sy, 0.0f, 0.0f),
                       zivc::makeFloat4(0.0f, 0.0f, sz, 0.0f),
                       zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f)};
  return m;
}

/*!
  \details No detailed description

  \param [in] sx No description.
  \param [in] sy No description.
  \param [in] sz No description.
  \return No description
  */
inline
Matrix4x4 getInvScalingMatrix(const float sx, const float sy, const float sz) noexcept
{
  const Matrix4x4 m = {zivc::makeFloat4(1.0f / sx, 0.0f, 0.0f, 0.0f),
                       zivc::makeFloat4(0.0f, 1.0f / sy, 0.0f, 0.0f),
                       zivc::makeFloat4(0.0f, 0.0f, 1.0f / sz, 0.0f),
                       zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f)};
  return m;
}

/*!
  \details No detailed description

  \param [in] r No description.
  \return No description
  */
inline
Matrix4x4 getRotationMatrix(const Quaternion r) noexcept
{
  const float4 q = r.data();
  const float4 m1 = zivc::makeFloat4(1.0f - 2.0f * (q.y * q.y + q.z * q.z),
                                     2.0f * (q.x * q.y - q.z * q.w),
                                     2.0f * (q.x * q.z + q.y * q.w),
                                     0.0f);
  const float4 m2 = zivc::makeFloat4(2.0f * (q.x * q.y + q.z * q.w),
                                     1.0f - 2.0f * (q.x * q.x + q.z * q.z),
                                     2.0f * (q.y * q.z - q.x * q.w),
                                     0.0f);
  const float4 m3 = zivc::makeFloat4(2.0f * (q.x * q.z - q.y * q.w),
                                     2.0f * (q.y * q.z + q.x * q.w),
                                     1.0f - 2.0f * (q.x * q.x + q.y * q.y),
                                     0.0f);
  const float4 m4 = zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f);
  return {m1, m2, m3, m4};
}

/*!
  \details No detailed description

  \param [in] r No description.
  \return No description
  */
inline
Matrix4x4 getInvRotationMatrix(const Quaternion r) noexcept
{
  const Matrix4x4 m = getRotationMatrix(r);
  return m.transpose();
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_TRANSFORMATION_INL_HPP */
