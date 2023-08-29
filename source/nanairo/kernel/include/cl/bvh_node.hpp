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

#ifndef NANAIRO_CL_BVH_NODE_HPP
#define NANAIRO_CL_BVH_NODE_HPP

namespace nanairo {

/*!
  \brief No brief description

  No detailed description.
  */
struct BvhNode
{
  float aabb_min_x_,
        aabb_min_y_,
        aabb_min_z_;
  float aabb_max_x_,
        aabb_max_y_,
        aabb_max_z_;
};

} /* namespace nanairo */

#endif /* NANAIRO_CL_BVH_NODE_HPP */
