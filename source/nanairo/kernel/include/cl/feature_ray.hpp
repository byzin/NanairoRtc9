/*!
  \file feature_ray.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_FEATURE_RAY_HPP
#define NANAIRO_CL_FEATURE_RAY_HPP

// Zivc
#include "zivc/cl/atomic.hpp"
#include "zivc/cl/geometric.hpp"
#include "zivc/cl/limits.hpp"
#include "zivc/cl/math.hpp"
#include "zivc/cl/numbers.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "camera.hpp"
#include "camera_info.hpp"
#include "context_info.hpp"
#include "hit_info.hpp"
#include "primary_sample_set.hpp"
#include "ray.hpp"

namespace nanairo {

inline
void getBasis(const float3 up, zivc::PrivatePtr<float3> tangent, zivc::PrivatePtr<float3> bitangent) noexcept
{
  const float3 tmp = (0.0001f < zivc::fabs(up.x)) ? zivc::makeFloat3(0.0f, 0.0f, 1.0f)
                                                  : zivc::makeFloat3(1.0f, 0.0f, 0.0f);
  const float3 t = zivc::cross(tmp, up);
  const float3 b = zivc::cross(up, t);

  *tangent = zivc::normalize(t);
  *bitangent = zivc::normalize(b);
}

inline
void generateFeatureRay(zivc::ConstGlobalPtr<PrimarySampleSet> sample_set,
                        zivc::ConstGlobalPtr<Ray> ray,
                        zivc::ConstGlobalPtr<HitInfo> hit_info,
                        zivc::GlobalPtr<Ray> feature_ray,
                        const zivc::uint32b gindex,
                        const zivc::uint32b n,
                        const nanairo::ContextInfo& context_info,
                        const nanairo::CameraInfo& camera_info,
                        const zivc::uint32b count) noexcept
{
  using FLimit = zivc::NumericLimits<float>;

  constexpr float distance = 0.0f;

  const nanairo::Ray r = ray[gindex];
  const nanairo::HitInfo hit = hit_info[gindex];
  if (!hit.hasHit())
    return;

  //
  const uint2 resolution = context_info.imageResolution();
  const float line_width = 1.0f;
  const float p_width = 2.0f * zivc::tan(camera_info.fov() / 2.0f) / static_cast<float>(resolution.y);

  const float width_start = distance * p_width * line_width;
  const float width_end = (distance + hit.distance()) * p_width * line_width;

  //
  const float h = hit.distance() * width_start / (width_end - width_start);
  const float3 to = r.origin() + r.direction() * hit.distance();
  const float3 top = r.origin() + r.direction() * h;
  const float3 c_y = zivc::normalize(to - top);
  float3 c_x = zivc::makeFloat3(0.0f, 0.0f, 0.0f),
         c_z = zivc::makeFloat3(0.0f, 0.0f, 0.0f);
  getBasis(c_y, &c_x, &c_z);

  //
  const float dc = h + hit.distance();
  const float sin_max2 = width_end * width_end / (width_end * width_end + dc * dc);
  const float cos_max = (zivc::min)(zivc::sqrt(1.0f - sin_max2), 1.0f - FLimit::epsilon());

  //
  const float2 samples = getSample2D(sample_set,
                                     gindex,
                                     n,
                                     ((count % 2) == 1) ? SampleSetUsage::kLightSampleX
                                                        : SampleSetUsage::kBsdfSampleX,
                                     count / 2);
  const float cos_theta = (1.0f - samples.x) + samples.x * cos_max;
  const float sin_theta = zivc::sqrt(1.0f - cos_theta * cos_theta);
  const float phi = 2.0f * zivc::numbers::kPi<float> * samples.y;

  const float r_start = h * sin_theta / cos_theta;
  const float r_end = dc * sin_theta / cos_theta;
  const float3 x_start = top + r_start * zivc::cos(phi) * c_x + r_start * zivc::sin(phi) * c_z + h * c_y;
  const float3 x_end = top + r_end * zivc::cos(phi) * c_x + r_end * zivc::sin(phi) * c_z + dc * c_y;
  const float3 dir = zivc::normalize(x_end - x_start);

  Ray feature_r{};
  feature_r.setOrigin(x_start);
  feature_r.setDirection(dir);
  feature_ray[gindex] = feature_r;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_FEATURE_RAY_HPP */
