/*!
  \file render_info.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_RENDER_INFO_HPP
#define NANAIRO_CL_RENDER_INFO_HPP

// Zivc
#include "zivc/cl/types.hpp"
// Nanairo
#include "nanairo_cl_config.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct RenderInfo
{
  //! Return the current frame
  uint16b currentFrame() const noexcept;

  //! Return the current iteration
  uint16b currentIteration() const noexcept;

  //! Initialize the render info
  void initialize() noexcept;

  //! Set the current frame number
  void setCurrentFrame(const uint16b frame) noexcept;

  //! Set the current iteration
  void setCurrentIteration(const uint16b iteration) noexcept;


  uint4 data0_;
};

//! Check if the two info has same values
bool operator==(const RenderInfo& lhs, const RenderInfo& rhs) noexcept;

} /* namespace nanairo */

#include "render_info-inl.hpp"

#endif /* NANAIRO_CL_RENDER_INFO_HPP */
