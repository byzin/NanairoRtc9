/*!
  \file png_writer.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "png_writer.hpp"
// Standard C++ library
#include <iostream>
#include <memory>
#include <span>
// spng
#include "spng/spng.h"
// Zisc
#include "zisc/binary_serializer.hpp"
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
#include "ldr_image.hpp"

namespace cmd {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
PngWriter::PngWriter([[maybe_unused]] zisc::pmr::memory_resource* mem_resource) noexcept
{
}

/*!
  \details No detailed description
  */
PngWriter::~PngWriter() noexcept
{
  destroy();
}

/*!
  \details No detailed description

  \return No description
  */
PngWriter::operator bool() const noexcept
{
  const bool flag = context_ != nullptr;
  return flag;
}

/*!
  \details No detailed description
  */
void PngWriter::destroy() noexcept
{
  if (context_ != nullptr) {
    spng_ctx_free(context_);
    context_ = nullptr;
  }
}

/*!
  \details No detailed description

  \param [in] width No description.
  \param [in] height No description.
  */
void PngWriter::initialize() noexcept
{
  // First destroy the previous writer
  destroy();

  //
  context_ = spng_ctx_new(SPNG_CTX_ENCODER);
  // check null
}

/*!
  \details No detailed description

  \param [in] image No description.
  \param [out] output No description.
  \return No description
  */
void PngWriter::write(const LdrImage& image, std::ostream* output) const noexcept
{
  //
  int result = spng_set_option(context_, SPNG_ENCODE_TO_BUFFER, 1);
  checkError(result);

  //
  constexpr zivc::uint8b bit_depth = 8;
  spng_ihdr ihdr{static_cast<zivc::uint32b>(image.width()),
                 static_cast<zivc::uint32b>(image.height()),
                 bit_depth,
                 SPNG_COLOR_TYPE_TRUECOLOR_ALPHA,
                 0,
                 0,
                 0};
  result = spng_set_ihdr(context_, &ihdr);
  checkError(result);

  //
  {
    const std::span buffer = image.data();
    const std::size_t size = sizeof(buffer[0]) * buffer.size();
    result = spng_encode_image(context_, buffer.data(), size, SPNG_FMT_RAW, SPNG_ENCODE_FINALIZE);
    checkError(result);
  }

  //
  {
    std::size_t buffer_size = 0;
    void* buffer = spng_get_png_buffer(context_, &buffer_size, &result);
    checkError(result);
    zisc::BinarySerializer::write(buffer, output, static_cast<std::streamsize>(buffer_size));
    std::free(buffer);
  }
}

/*!
  \details No detailed description

  \param [in] no No description.
  */
void PngWriter::checkError(const int no) noexcept
{
  if (no != SPNG_OK) {
    std::cerr << "  PngWriter error: " << spng_strerror(no) << std::endl;
  }
}

} /* namespace cmd */
