/*!
  \file bvh_info-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_BVH_INFO_INL_HPP
#define NANAIRO_CL_BVH_INFO_INL_HPP

#include "bvh_info.hpp"
// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "cl/matrix_4x4.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \return No description
  */
inline
zivc::uint32b BvhInfo::faceOffset() const noexcept
{
  return data0_.w;
}

/*!
  \details No detailed description

  \return No description
  */
inline
zivc::uint32b BvhInfo::geometryOffset() const noexcept
{
  return data0_.z;
}

/*!
  \details No detailed description

  \return No description
  */
inline
Matrix4x4 BvhInfo::invTransformation() const noexcept
{
  return {m1_, m2_, m3_, m4_};
}

/*!
  \details No detailed description

  \return No description
  */
inline
zivc::uint32b BvhInfo::maxDepthLevel() const noexcept
{
  return data0_.x;
}

/*!
  \details No detailed description

  \return No description
  */
inline
zivc::uint32b BvhInfo::numOfVirtualLeaves() const noexcept
{
  return data0_.y;
}

/*!
  \details No detailed description

  \param [in] offset No description.
  */
inline
void BvhInfo::setFaceOffset(const zivc::uint32b offset) noexcept
{
  data0_.w = offset;
}

/*!
  \details No detailed description

  \param [in] offset No description.
  */
inline
void BvhInfo::setGeometryOffset(const zivc::uint32b offset) noexcept
{
  data0_.z = offset;
}

/*!
  \details No detailed description

  \param [in] m No description.
  */
inline
void BvhInfo::setInvTransformation(const Matrix4x4 m) noexcept
{
  m1_ = m.m1_;
  m2_ = m.m2_;
  m3_ = m.m3_;
  m4_ = m.m4_;
}

/*!
  \details No detailed description

  \param [in] level No description.
  */
inline
void BvhInfo::setMaxDepthLevel(const zivc::uint32b level) noexcept
{
  data0_.x = level;
}

/*!
  \details No detailed description

  \param [in] level No description.
  */
inline
void BvhInfo::setNumOfVirtualLeaves(const zivc::uint32b n) noexcept
{
  data0_.y = n;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_BVH_INFO_INL_HPP */
