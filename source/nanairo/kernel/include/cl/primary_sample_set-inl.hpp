/*!
  \file primary_sample_set-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_PRIMARY_SAMPLE_SET_INL_HPP
#define NANAIRO_CL_PRIMARY_SAMPLE_SET_INL_HPP

#include "primary_sample_set.hpp"
// Zivc
#include "zivc/cl/limits.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
#include "zivc/cl/vector_data.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"
#include "utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
float PrimarySampleSet::operator[](const size_t index) const noexcept
{
  return get(index);
}

/*!
  \details No detailed description

  \return No description
  */
inline
constexpr size_t PrimarySampleSet::capacity() noexcept
{
  return kCapacity;
}

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
float PrimarySampleSet::get(const size_t index) const noexcept
{
  return decode(index);
}

/*!
  \details No detailed description

  \param [in] sample No description.
  \param [in] index No description.
  */
inline
void PrimarySampleSet::set(const float sample, const size_t index) noexcept
{
  encode(sample, index);
}

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
float PrimarySampleSet::decode(const size_t index) const noexcept
{
  const size_t i = index % 4;
  uint32b sample = 0;
  {
    const uint3 mask = getMask(i);
    const uint3 d = mask & getData();
    const uint32b vx = (i == 0) ? (d.x << 8) :
                       (i == 1) ? (d.x >> 16u)
                                : 0u;
    const uint32b vy = (i == 1) ? (d.y << 16u) :
                       (i == 2) ? (d.y >> 8u)
                                : 0u;
    const uint32b vz = (i == 2) ? (d.z << 24u) :
                       (i == 3) ? d.z
                                : 0u;
    sample = vx | vy | vz;
  }
  return Utility::mapTo01(sample);
}

/*!
  \details No detailed description

  \param [in] sample No description.
  \param [in] index No description.
  */
inline
void PrimarySampleSet::encode(const float sample, const size_t index) noexcept
{
  //
  const size_t i = index % 4;
  const uint32b value = Utility::mapToInt(sample);
  uint3 d = getData();
  {
    const uint32b vx = (i == 0) ? (value >> 8u) :
                       (i == 1) ? (value << 16u)
                                : 0u;
    const uint32b vy = (i == 1) ? (value >> 16u) :
                       (i == 2) ? (value << 8u)
                                : 0u;
    const uint32b vz = (i == 2) ? (value >> 24u) :
                       (i == 3) ? value
                                : 0u;
    const uint3 v = zivc::makeUInt3(vx, vy, vz);
    const uint3 mask = getMask(i);
    d = (mask & v) | (~mask & d);
  }
  setData(d);
}

/*!
  \details No detailed description

  \return No description
  */
inline
uint3 PrimarySampleSet::getData() const noexcept
{
  const uint3 value = zivc::makeUInt3(data0_, data1_, data2_);
  return value;
}

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
uint3 PrimarySampleSet::getMask(const size_t i) noexcept
{
  const uint32b x = (i == 0) ? 0b0000'0000'1111'1111'1111'1111'1111'1111u :
                    (i == 1) ? 0b1111'1111'0000'0000'0000'0000'0000'0000u
                             : 0b0000'0000'0000'0000'0000'0000'0000'0000u;
  const uint32b y = (i == 1) ? 0b0000'0000'0000'0000'1111'1111'1111'1111u :
                    (i == 2) ? 0b1111'1111'1111'1111'0000'0000'0000'0000u
                             : 0b0000'0000'0000'0000'0000'0000'0000'0000u;
  const uint32b z = (i == 2) ? 0b0000'0000'0000'0000'0000'0000'1111'1111u :
                    (i == 3) ? 0b1111'1111'1111'1111'1111'1111'0000'0000u
                             : 0b0000'0000'0000'0000'0000'0000'0000'0000u;
  const uint3 mask = zivc::makeUInt3(x, y, z);
  return mask;
}

/*!
  \details No detailed description

  \param [in] value No description.
  */
inline
void PrimarySampleSet::setData(const uint3 value) noexcept
{
  data0_ = value.x;
  data1_ = value.y;
  data2_ = value.z;
}

/*!
  \details No detailed description

  \return No description
  */
inline
size_t calcSampleSetSize(const size_t max_bounces) noexcept
{
  const size_t n = static_cast<size_t>(SampleSetUsage::kMax) +
                   max_bounces * static_cast<size_t>(SampleSetUsage::kBounceOffset);
  constexpr size_t cap = PrimarySampleSet::capacity();
  const size_t size = (n + cap - 1) / cap;
  return size;
}

/*!
  \details No detailed description

  \param [in] ptr No description.
  \param [in] global_index No description.
  \param [in] global_offset No description.
  \param [in] usage No description.
  \param [in] bounce No description.
  \return No description
  */
inline
float2 getSample2D(const zivc::ConstGlobalPtr<PrimarySampleSet> ptr,
                   const size_t global_index,
                   const size_t global_offset,
                   const SampleSetUsage usage,
                   const size_t bounce) noexcept
{
  const size_t d = static_cast<size_t>(usage) +
                   bounce * static_cast<size_t>(SampleSetUsage::kBounceOffset);
  const size_t set_index = d / PrimarySampleSet::capacity();
  const size_t sample_index = d - set_index * PrimarySampleSet::capacity();
  const PrimarySampleSet set = getSampleSet(ptr, global_index, global_offset, set_index);
  const float2 sample = zivc::makeFloat2(set.get(sample_index),
                                         set.get(sample_index + 1));
  return sample;
}

/*!
  \details No detailed description

  \param [in] ptr No description.
  \param [in] global_index No description.
  \param [in] global_offset No description.
  \param [in] set_index No description.
  \return No description
  */
inline
PrimarySampleSet getSampleSet(zivc::ConstGlobalPtr<PrimarySampleSet> ptr,
                              const size_t global_index,
                              const size_t global_offset,
                              const size_t set_index) noexcept
{
  const size_t index = global_index + set_index * global_offset;
  return ptr[index];
}

/*!
  \details No detailed description

  \param [out] ptr No description.
  \param [in] set No description.
  \param [in] global_index No description.
  \param [in] global_offset No description.
  \param [in] set_index No description.
  */
inline
void setSampleSet(zivc::GlobalPtr<PrimarySampleSet> ptr,
                  const PrimarySampleSet set,
                  const size_t global_index,
                  const size_t global_offset,
                  const size_t set_index) noexcept
{
  const size_t index = global_index + set_index * global_offset;
  ptr[index] = set;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_PRIMARY_SAMPLE_SET_INL_HPP */
