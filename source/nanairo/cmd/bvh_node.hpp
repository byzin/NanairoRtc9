/*!
  \file bvh_node.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CMD_BVH_NODE_HPP
#define NANAIRO_CMD_BVH_NODE_HPP

// Standard C++ library
#include <array>
#include <cstdint>

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct BvhInfo
{
  std::uint32_t max_level_ = 0;
  std::uint32_t num_of_virtual_leaves_ = 0;
};

/*!
  \brief No brief description

  No detailed description.
  */
struct BvhNode
{
  std::array<float, 6> aabb_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CMD_BVH_NODE_HPP */
