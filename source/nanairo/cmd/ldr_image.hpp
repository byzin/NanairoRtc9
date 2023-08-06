/*!
  \file ldr_image.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_LDR_IMAGE_HPP
#define NANAIRO_CMD_LDR_IMAGE_HPP 1

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
  \brief No brief description

  No detailed description.
  */
class LdrImage
{
 public:
  //! Create a ldr image
  LdrImage(zisc::pmr::memory_resource* mem_resource) noexcept;

  //! Destroy the ldr image
  ~LdrImage() noexcept;


  //! Return the reference to the underlying pixel value
  zivc::cl::uchar4& operator[](const std::size_t index) noexcept;

  //! Return the reference to the underlying pixel value
  const zivc::cl::uchar4& operator[](const std::size_t index) const noexcept;

  //! Check if the image is initialized
  explicit operator bool() const noexcept;


  //! Return the underlying data
  std::span<zivc::cl::uchar4> data() noexcept;

  //! Return the underlying data
  std::span<const zivc::cl::uchar4> data() const noexcept;

  //! Destroy the image
  void destroy() noexcept;

  //! Return the reference to the underlying pixel value
  zivc::cl::uchar4& get(const std::size_t index) noexcept;

  //! Return the reference to the underlying pixel value
  const zivc::cl::uchar4& get(const std::size_t index) const noexcept;

  //! Return the height of the image
  size_t height() const noexcept;

  //! Initialize the image
  void initialize(const std::size_t width, const std::size_t height) noexcept;

  //! Return the width of the image
  size_t width() const noexcept;

 private:
  std::vector<zivc::cl::uchar4> data_;
  std::size_t width_;
  std::size_t height_;
};

} /* namespace cmd */

#include "ldr_image-inl.hpp"

#endif /* NANAIRO_CMD_LDR_IMAGE_HPP */
