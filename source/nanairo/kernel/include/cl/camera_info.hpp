/*!
  \file camera_info.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CAMERA_INFO_HPP
#define NANAIRO_CL_CAMERA_INFO_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "matrix_4x4.hpp"
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct CameraInfo
{
  //! Return the y fov
  float fov() const noexcept;

  //! Set the y fov
  void setFov(const float fov) noexcept;

  //! Set the transformation matrix
  void setTransformation(const Matrix4x4 m) noexcept;

  //! Return the transformation matrix
  Matrix4x4 transformation() const noexcept;


  float4 data0_;
  float4 m1_,
         m2_,
         m3_,
         m4_;
};

//! Check if the two info has same values
bool operator==(const CameraInfo& lhs, const CameraInfo& rhs) noexcept;

} /* namespace nanairo */

#include "camera_info-inl.hpp"

#endif /* NANAIRO_CL_CAMERA_INFO_HPP */
