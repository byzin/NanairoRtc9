/*!
  \file bvh_node_stack.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_BVH_NODE_STACK_HPP
#define NANAIRO_CL_BVH_NODE_STACK_HPP

// Zivc
#include "zivc/cl/types.hpp"

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct BvhNodeStack
{
  static constexpr size_t kSize = 8;


  //! Return the size of the stack
  static constexpr size_t size() noexcept
  {
    return kSize;
  }


  uint2 stack_[kSize];
};

} /* namespace nanairo */

#endif /* NANAIRO_CL_BVH_NODE_STACK_HPP */
