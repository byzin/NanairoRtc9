/*!
  \file bvh_info.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_BVH_INFO_HPP
#define NANAIRO_CL_BVH_INFO_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "cl/matrix_4x4.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct BvhInfo
{
  //!
  zivc::uint32b faceOffset() const noexcept;

  //!
  zivc::uint32b geometryOffset() const noexcept;

  //! Return the inv transformation matrix
  Matrix4x4 invTransformation() const noexcept;

  //! Return the max depth level of the bvh
  zivc::uint32b maxDepthLevel() const noexcept;

  //! Return the number of virtual leaves
  zivc::uint32b numOfVirtualLeaves() const noexcept;

  //!
  void setFaceOffset(const zivc::uint32b offset) noexcept;

  //!
  void setGeometryOffset(const zivc::uint32b offset) noexcept;

  //! Set the inv transformation matrix
  void setInvTransformation(const Matrix4x4 m) noexcept;

  //! Set the max depth level of the bvh
  void setMaxDepthLevel(const zivc::uint32b level) noexcept;

  //! Set the number of virtual leaves
  void setNumOfVirtualLeaves(const zivc::uint32b n) noexcept;


  uint4 data0_;
  float4 m1_,
         m2_,
         m3_,
         m4_;
};

} /* namespace nanairo */

#include "bvh_info-inl.hpp"

#endif /* NANAIRO_CL_BVH_INFO_HPP */
