/*!
  \file ldr_image-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_LDR_IMAGE_INL_HPP
#define NANAIRO_CMD_LDR_IMAGE_INL_HPP 1

#include "ldr_image.hpp"
// Standard C++ library
#include <span>
// Zivc
#include "zivc/zivc.hpp"

namespace cmd {

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
zivc::cl::uchar4& LdrImage::operator[](const std::size_t index) noexcept
{
  return get(index);
}

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
const zivc::cl::uchar4& LdrImage::operator[](const std::size_t index) const noexcept
{
  return get(index);
}

/*!
  \details No detailed description

  \return No description
  */
inline
LdrImage::operator bool() const noexcept
{
  const bool flag = !data_.empty();
  return flag;
}

/*!
  \details No detailed description

  \return No description
  */
inline
std::span<zivc::cl::uchar4> LdrImage::data() noexcept
{
  return data_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
std::span<const zivc::cl::uchar4> LdrImage::data() const noexcept
{
  return data_;
}


/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
zivc::cl::uchar4& LdrImage::get(const std::size_t index) noexcept
{
  std::span d = data();
  return d[index];
}

/*!
  \details No detailed description

  \param [in] index No description.
  \return No description
  */
inline
const zivc::cl::uchar4& LdrImage::get(const std::size_t index) const noexcept
{
  const std::span d = data();
  return d[index];
}

/*!
  \details No detailed description

  \return No description
  */
inline
std::size_t LdrImage::height() const noexcept
{
  return height_;
}

/*!
  \details No detailed description

  \return No description
  */
inline
std::size_t LdrImage::width() const noexcept
{
  return width_;
}

} /* namespace cmd */

#endif /* NANAIRO_CMD_LDR_IMAGE_INL_HPP */
