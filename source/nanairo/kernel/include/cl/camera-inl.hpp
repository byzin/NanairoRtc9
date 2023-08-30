/*!
  \file camera-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CAMERA_INL_HPP
#define NANAIRO_CL_CAMERA_INL_HPP

#include "camera.hpp"
// Zivc
#include "zivc/cl/math.hpp"
#include "zivc/cl/numbers.hpp"
#include "zivc/cl/geometric.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "camera_info.hpp"
#include "cl/matrix_4x4.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] info No description.
  \param [in] resolution No description.
  \param [in] index No description.
  \param [in] samples No description.
  \return No description
  */
inline
Ray generateRayPerspective(const CameraInfo& info,
                           const uint2 resolution,
                           const zivc::uint32b index,
                           const float2 samples) noexcept
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b y = index / resolution.x;
  const uint32b x = index - resolution.x * y;
  const float2 coord = zivc::makeFloat2(
      (static_cast<float>(x) + samples.x) / static_cast<float>(resolution.x),
      (static_cast<float>(y) + samples.y) / static_cast<float>(resolution.y)) - 0.5f;

  constexpr float default_sensor_size = 0.024f;
  const float aspect_ratio = static_cast<float>(resolution.x) / static_cast<float>(resolution.y);
  const float2 sensor_size = zivc::makeFloat2(default_sensor_size * aspect_ratio, default_sensor_size);

  float4 o = zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f);
  float4 d = zivc::makeFloat4(coord.x * sensor_size.x,
                              -coord.y * sensor_size.y,
                              -sensor_size.y / (2.0f * zivc::tan(info.fov() / 2.0f)),
                              0.0f);
  {
    const Matrix4x4 m = info.transformation();
    o = m * o;
    d = m * d;
    d = zivc::normalize3(d);
  }

  return {o.x, o.y, o.z, d.x, d.y, d.z};
}

/*!
  \details No detailed description

  \param [in] info No description.
  \param [in] resolution No description.
  \param [in] index No description.
  \param [in] samples No description.
  \return No description
  */
inline
Ray generateRayLatLong(const CameraInfo& info,
                       const uint2 resolution,
                       const zivc::uint32b index,
                       const float2 samples) noexcept
{
  using zivc::int32b;
  using zivc::uint32b;

  const uint32b y = index / resolution.x;
  const uint32b x = index - resolution.x * y;
  const float2 coord = zivc::makeFloat2(
      (static_cast<float>(x) + samples.x) / static_cast<float>(resolution.x),
      (static_cast<float>(y) + samples.y) / static_cast<float>(resolution.y)) - 0.5f;

  const float theta = (1.0f - coord.y) * zivc::numbers::kPi<float>;
  const float phi = (2.0f * coord.x - 1.0f) * zivc::numbers::kPi<float>;
  const float3 dir = zivc::makeFloat3(zivc::sin(theta) * zivc::sin(phi),
                                      zivc::cos(theta),
                                      zivc::sin(theta) * zivc::cos(phi));

  float4 o = zivc::makeFloat4(0.0f, 0.0f, 0.0f, 1.0f);
  float4 d = zivc::makeFloat4(dir.x, dir.y, dir.z, 0.0f);
  {
    const Matrix4x4 m = info.transformation();
    o = m * o;
    d = m * d;
    d = zivc::normalize3(d);
  }

  return {o.x, o.y, o.z, d.x, d.y, d.z};
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_CAMERA_INL_HPP */
