/*!
  \file correlated_multi_jittered_engine.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_HPP
#define NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.

  \tparam kRootN No description.
  */
template <uint32b kRootN>
class CorrelatedMultiJitteredEngine
{
 public:
  //! Generate an 1d [0, 1) float random number
  static float draw1D(const uint32b s, const uint32b p) noexcept;

  //! Generate a 2d [0, 1) float random numbers
  static float2 draw2D(const uint32b s, const uint32b p) noexcept;

  //! Return the period
  static constexpr uint32b getPeriod() noexcept;

 private:
  //!
  template <typename T>
  static T hash(const T i, const T p) noexcept;

  //!
  static constexpr uint32b makeWMask(const uint32b w) noexcept;

  //!
  template <typename T, uint32b l>
  static T permute(const T i, const T p) noexcept;

  //!
  template <typename T, uint32b w>
  static T permuteImpl(const T i, const T p) noexcept;
};

// Type aliases
using CmjEngine4x4 = CorrelatedMultiJitteredEngine<4>;

} /* namespace nanairo */

#include "correlated_multi_jittered_engine-inl.hpp"

#endif /* NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_HPP */
