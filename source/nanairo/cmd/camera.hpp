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

#ifndef NANAIRO_CMD_CAMERA_HPP
#define NANAIRO_CMD_CAMERA_HPP

// Standard C++ library
#include <array>
// Nanairo
#include "utility.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
class Camera
{
 public:
  //! Create a camera
  Camera() noexcept {}


  float yfov_ = 0.0f;
  std::array<float, 16> transformation_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_CAMERA_HPP */
