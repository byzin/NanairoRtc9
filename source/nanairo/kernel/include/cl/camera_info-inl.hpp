/*!
  \file camera_info-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CAMERA_INFO_INL_HPP
#define NANAIRO_CL_CAMERA_INFO_INL_HPP

#include "camera_info.hpp"
// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "matrix_4x4.hpp"
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
float CameraInfo::fov() const noexcept
{
  return data0_.x;
}

/*!
  \details No detailed description

  \param [in] fov No description.
  */
inline
void CameraInfo::setFov(const float fov) noexcept
{
  data0_.x = fov;
}

/*!
  \details No detailed description

  \param [in] m No description.
  */
inline
void CameraInfo::setTransformation(const Matrix4x4 m) noexcept
{
  m1_ = m.m1_;
  m2_ = m.m2_;
  m3_ = m.m3_;
  m4_ = m.m4_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Matrix4x4 CameraInfo::transformation() const noexcept
{
  return {m1_, m2_, m3_, m4_};
}

/*!
  \details No detailed description

  \param [in] lhs No description.
  \param [in] rhs No description.
  \return No description
  */
inline
bool operator==(const CameraInfo& lhs, const CameraInfo& rhs) noexcept
{
  const bool result = (lhs.data0_.x == rhs.data0_.x) &&
                      (lhs.data0_.y == rhs.data0_.y) &&
                      (lhs.data0_.z == rhs.data0_.z) &&
                      (lhs.data0_.w == rhs.data0_.w) &&
                      (lhs.m1_.x == rhs.m1_.x) &&
                      (lhs.m1_.y == rhs.m1_.y) &&
                      (lhs.m1_.z == rhs.m1_.z) &&
                      (lhs.m1_.w == rhs.m1_.w) &&
                      (lhs.m2_.x == rhs.m2_.x) &&
                      (lhs.m2_.y == rhs.m2_.y) &&
                      (lhs.m2_.z == rhs.m2_.z) &&
                      (lhs.m2_.w == rhs.m2_.w) &&
                      (lhs.m3_.x == rhs.m3_.x) &&
                      (lhs.m3_.y == rhs.m3_.y) &&
                      (lhs.m3_.z == rhs.m3_.z) &&
                      (lhs.m3_.w == rhs.m3_.w) &&
                      (lhs.m4_.x == rhs.m4_.x) &&
                      (lhs.m4_.y == rhs.m4_.y) &&
                      (lhs.m4_.z == rhs.m4_.z) &&
                      (lhs.m4_.w == rhs.m4_.w);
  return result;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_CAMERA_INFO_INL_HPP */
