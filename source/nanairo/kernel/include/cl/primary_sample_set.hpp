/*!
  \file primary_sample_set.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_PRIMARY_SAMPLE_SET_HPP
#define NANAIRO_CL_PRIMARY_SAMPLE_SET_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
enum class SampleSetUsage : uint32b
{
  kImagePlaneX = 0,
  kImagePlaneY,
  // Start of an iteration
  kBsdfSampleX,
  kBsdfSampleY,
  kLightSampleX,
  kLightSampleY,
  // End of an iteration
  kMax,
  kBounceOffset = 4 //!< The number of samples during an iteration
};

/*!
  \brief No brief description

  No detailed description.
  */
class PrimarySampleSet
{
 public:
  //! Return the sample by the given index
  float operator[](const size_t index) const noexcept;


  //! Return the capacity of this set
  static constexpr size_t capacity() noexcept;

  //! Return the sample by the given index
  float get(const size_t index) const noexcept;

  //! Set a sample
  void set(const float sample, const size_t index) noexcept;

  //!
  static size_t getSetOffset(const size_t max_bounces) noexcept;

 private:
  //! Decode the sample from the underlying data
  float decode(const size_t index) const noexcept;

  //! Encode a sample into the underlying data
  void encode(const float sample, const size_t index) noexcept;

  //!
  static uint3 getMask(const size_t i) noexcept;

  //!
  uint32b getValue(const size_t index) const noexcept;


  static constexpr size_t kCapacity = 16;


  uint32b data_[12]; //!< 16 float samples are encoded into uint[12]
};

} /* namespace nanairo */

#include "primary_sample_set-inl.hpp"

#endif /* NANAIRO_CL_PRIMARY_SAMPLE_SET_HPP */
