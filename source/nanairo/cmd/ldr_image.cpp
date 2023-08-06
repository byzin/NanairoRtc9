/*!
  \file ldr_image.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "ldr_image.hpp"

// Standard C++ library
#include <cstddef>
#include <memory>
#include <span>
#include <vector>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"
// Zivc
#include "zivc/zivc.hpp"

namespace cmd {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
LdrImage::LdrImage([[maybe_unused]] zisc::pmr::memory_resource* mem_resource) noexcept
{
}

/*!
  \details No detailed description
  */
LdrImage::~LdrImage() noexcept
{
  destroy();
}

/*!
  \details No detailed description
  */
void LdrImage::destroy() noexcept
{
  data_.clear();
}

/*!
  \details No detailed description

  \param [in] width No description.
  \param [in] height No description.
  */
void LdrImage::initialize(const std::size_t width, const std::size_t height) noexcept
{
  // First destroy the previous data
  destroy();

  width_ = width;
  height_ = height;
  data_.resize(width * height);
}

} /* namespace cmd */
