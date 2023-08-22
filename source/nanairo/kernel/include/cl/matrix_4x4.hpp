/*!
  \file matrix_4x4.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_MATRIX_4X4_HPP
#define NANAIRO_CL_MATRIX_4X4_HPP

// Zivc
#include "zivc/cl/types.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct Matrix4x4
{
  //! Return the identity matrix
  static Matrix4x4 identity() noexcept;

  //! Set the matrix values
  void set(const float4 m1, const float4 m2, const float4 m3, const float4 m4) noexcept;

  //! Return the transpose of the matrix
  Matrix4x4 transpose() const noexcept;


  float4 m1_, //!< Represent the row1
         m2_, //!< Represent the row2
         m3_, //!< Represent the row3
         m4_; //!< Represent the row4
};

//! Multiply the matrix with the vector
float4 operator*(const Matrix4x4 lhs, const float4 rhs) noexcept;

//! Multiply the matrices
Matrix4x4 operator*(const Matrix4x4 lhs, const Matrix4x4 rhs) noexcept;

} /* namespace nanairo */

#include "matrix_4x4-inl.hpp"

#endif /* NANAIRO_CL_MATRIX_4X4_HPP */
