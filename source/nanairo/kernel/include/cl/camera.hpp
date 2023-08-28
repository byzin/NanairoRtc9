/*!
  \file camera.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CAMERA_HPP
#define NANAIRO_CL_CAMERA_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "camera_info.hpp"
#include "ray.hpp"

namespace nanairo {

//! Generate a ray from the camera
Ray generateRay(const CameraInfo& info,
                const uint2 resolution,
                const zivc::uint32b index,
                const float2 samples) noexcept;

} /* namespace nanairo */

#include "camera-inl.hpp"

#endif /* NANAIRO_CL_CAMERA_HPP */
