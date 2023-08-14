/*!
  \file png_writer.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_PNG_WRITER_HPP
#define NANAIRO_CMD_PNG_WRITER_HPP 1

// Standard C++ library
#include <cstddef>
#include <memory>
#include <ostream>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"

// Forward declaration
typedef struct spng_ctx spng_ctx;

namespace nanairo {

// Forward declaration
class LdrImage;

/*!
  \brief No brief description

  No detailed description.
  */
class PngWriter
{
 public:
  //! Create a writer
  PngWriter(zisc::pmr::memory_resource* mem_resource) noexcept;

  //! Destroy the writer
  ~PngWriter() noexcept;


  //! Check if the writer is initialized
  explicit operator bool() const noexcept;


  //! Destroy the writer
  void destroy() noexcept;

  //! Initialize the writer
  void initialize() noexcept;

  //! Encode the LDR image into a png format and write to the output stream
  void write(const LdrImage& image, std::ostream* output) const noexcept;

 private:
  //! Check if the result has any error
  static void checkError(const int no) noexcept;


  spng_ctx* context_ = nullptr;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_PNG_WRITER_HPP */
