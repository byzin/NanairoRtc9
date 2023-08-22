/*!
  \file quaternion.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_QUATERNION_HPP
#define NANAIRO_CL_QUATERNION_HPP

// Zivc
#include "zivc/cl/types.hpp"

namespace nanairo {

/*!
  \brief The representation of a rotation quaternion

  No detailed description.
  */
struct Quaternion
{
  //! Return the underlying data
  float4 data() const noexcept;

  //! Compute the conjugate of the quaternion
  Quaternion getConjugate() const noexcept;

  //! Return the identity quaternion
  static Quaternion identity() noexcept;

  //! Compute the inverse of the quaternion
  Quaternion invert() const noexcept;

  //! Rotate the given vector using the quaternion
  float3 rotate(const float3 vec) const noexcept;

  //! Set the rotation data
  void set(const float x, const float y, const float z, const float angle) noexcept;

  //! Set the rotation data
  void set(const float4 data) noexcept;


  float4 data_;
};

//! Compute the multiplication of quaternions
Quaternion operator*(const Quaternion lhs, const Quaternion rhs) noexcept;

} /* namespace nanairo */

#include "quaternion-inl.hpp"

#endif /* NANAIRO_CL_QUATERNION_HPP */
