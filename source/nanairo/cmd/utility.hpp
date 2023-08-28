/*!
  \file utility.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_UTILITY_HPP
#define NANAIRO_CMD_UTILITY_HPP

// Standard C++ library
#include <cstdint>

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct U3
{
  std::uint32_t x_ = 0,
                y_ = 0,
                z_ = 0;
};

/*!
  \brief No brief description

  No detailed description.
  */
struct F2
{
  float x_ = 0.0f,
        y_ = 0.0f;
};

/*!
  \brief No brief description

  No detailed description.
  */
struct F3
{
  float x_ = 0.0f,
        y_ = 0.0f,
        z_ = 0.0f;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_UTILITY_HPP */
