/*!
  \file transformation.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_TRANSFORMATION_HPP
#define NANAIRO_CL_TRANSFORMATION_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "matrix_4x4.hpp"
#include "quaternion.hpp"

namespace nanairo {

//! Return the translation matrix
Matrix4x4 getTranslationMatrix(const float tx, const float ty, const float tz) noexcept;

//! Return the inverse of the translation matrix
Matrix4x4 getInvTranslationMatrix(const float tx, const float ty, const float tz) noexcept;

//! Return the scaling matrix
Matrix4x4 getScalingMatrix(const float sx, const float sy, const float sz) noexcept;

//! Return the inverse of the scaling matrix
Matrix4x4 getInvScalingMatrix(const float sx, const float sy, const float sz) noexcept;

//! Return the rotation matrix
Matrix4x4 getRotationMatrix(const Quaternion r) noexcept;

//! Return the inverse of the rotation matrix
Matrix4x4 getInvRotationMatrix(const Quaternion r) noexcept;

} /* namespace nanairo */

#include "transformation-inl.hpp"

#endif /* NANAIRO_CL_TRANSFORMATION_HPP */
