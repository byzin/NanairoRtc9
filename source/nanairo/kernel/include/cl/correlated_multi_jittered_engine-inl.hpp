/*!
  \file correlated_multi_jittered_engine-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_INL_HPP
#define NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_INL_HPP

#include "correlated_multi_jittered_engine.hpp"
// Zivc
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"
#include "utility.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] s No description.
  \param [in] p No description.
  \return No description
  */
template <uint32b kRootN> inline
float CorrelatedMultiJitteredEngine<kRootN>::draw1D(const uint32b s, const uint32b p) noexcept
{
  constexpr uint32b n = getPeriod();

  const uint32b sp = permute<uint32b, n>(s, p * 0x68bc21ebu);
  float x = Utility::mapTo01(hash<uint32b>(s, p * 0x967a889bu));

  // Random jitter
  constexpr float inv_n = 1.0f / static_cast<float>(n);
  x = inv_n * (static_cast<float>(sp) + x);

  return x;
}

/*!
  \details No detailed description

  \param [in] s No description.
  \param [in] p No description.
  \return No description
  */
template <uint32b kRootN> inline
float2 CorrelatedMultiJitteredEngine<kRootN>::draw2D(const uint32b s, const uint32b p) noexcept
{
  constexpr uint32b n = getPeriod();

  const uint32b sp = permute<uint32b, n>(s, p * 0x51633e2du);

  const uint32b s_div = sp / kRootN;
  const uint32b s_mod = sp  - s_div * kRootN;

  const uint2 sp2 = permute<uint2, kRootN>(zivc::makeUInt2(s_mod, s_div),
                                           zivc::makeUInt2(0x68bc21ebu, 0x02e5be93u) * p);
  float2 v = Utility::mapTo01(hash<uint2>(zivc::makeUInt2(sp, sp), 
                                          zivc::makeUInt2(0x967a889bu, 0x368cc8b7u) * p));

  // Random jitter
  constexpr float inv_n = 1.0f / static_cast<float>(n);
  constexpr float inv_root_n = 1.0f / static_cast<float>(kRootN);
  v.x = inv_root_n * (static_cast<float>(sp2.x) + inv_root_n * (static_cast<float>(sp2.y) + v.x));
  v.y = inv_n * (static_cast<float>(sp) + v.y);

  return v;
}

/*!
  \details No detailed description

  \return No description
  */
template <uint32b kRootN> inline
constexpr uint32b CorrelatedMultiJitteredEngine<kRootN>::getPeriod() noexcept
{
  const uint32b period = kRootN * kRootN;
  return period;
}

/*!
  \details No detailed description

  \tparam T No description.
  \param [in] i No description.
  \param [in] p No description.
  \return No description
  */
template <uint32b kRootN> template <typename T> inline
T CorrelatedMultiJitteredEngine<kRootN>::hash(const T i, const T p) noexcept
{
  T ii = i;
  ii ^= p;
  ii ^= ii >> 17;
  ii ^= ii >> 10;
  ii *= 0xb36534e5u;
  ii ^= ii >> 12;
  ii ^= ii >> 21;
  ii *= 0x93fc4795u;
  ii ^= 0xdf6e307fu;
  ii ^= ii >> 17;
  ii *= 1u | p >> 18;
  return ii;
}

/*!
  \details No detailed description

  \param [in] w No description.
  \return No description
  */
template <uint32b kRootN> inline
constexpr uint32b CorrelatedMultiJitteredEngine<kRootN>::makeWMask(uint32b w) noexcept
{
  w |= w >> 1;
  w |= w >> 2;
  w |= w >> 4;
  w |= w >> 8;
  w |= w >> 16;
  return w;
}

/*!
  \details No detailed description

  \tparam T No description.
  \tparam l No description.
  \param [in] i No description.
  \param [in] p No description.
  \return No description
  */
template <uint32b kRootN> template <typename T, uint32b l> inline
T CorrelatedMultiJitteredEngine<kRootN>::permute(const T i, const T p) noexcept
{
  constexpr bool is_power_of_2 = (1u < l) && ((l & (l - 1)) == 0);
  static_assert(is_power_of_2, "The l isn't power of 2.");
  constexpr uint32b w = makeWMask(l - 1u);
  T ii = permuteImpl<T, w>(i, p);
  ii = (ii + p) & w;
  return ii;
}

/*!
  \details No detailed description

  \tparam T No description.
  \tparam w No description.
  \param [in] i No description.
  \param [in] p No description.
  \return No description
  */
template <uint32b kRootN> template <typename T, uint32b w> inline
T CorrelatedMultiJitteredEngine<kRootN>::permuteImpl(const T i, const T p) noexcept
{
  T ii = i;
  ii ^= p;
  ii *= 0xe170893du;
  ii ^= p >> 16;
  ii ^= (ii & w) >> 4;
  ii ^= p >> 8;
  ii *= 0x0929eb3fu;
  ii ^= p >> 23;
  ii ^= (ii & w) >> 1;
  ii *= 1 | p >> 27;
  ii *= 0x6935fa69u;
  ii ^= (ii & w) >> 11;
  ii *= 0x74dcb303u;
  ii ^= (ii & w) >> 2;
  ii *= 0x9e501cc3u;
  ii ^= (ii & w) >> 2;
  ii *= 0xc860a3dfu;
  ii &= w;
  ii ^= ii >> 5;
  return ii;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_CORRELATED_MULTI_JITTERED_ENGINE_INL_HPP */
