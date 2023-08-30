/*!
  \file mesh.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "mesh.hpp"
// Standard C++ library
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <ostream>
#include <vector>
#include <utility>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
#include "zivc/kernel_set/kernel_set-ray_cast_kernel.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
Mesh::Mesh(zisc::pmr::memory_resource* mem_resource) noexcept :
    vertices_{decltype(vertices_)::allocator_type{mem_resource}},
    normals_{decltype(normals_)::allocator_type{mem_resource}},
    texcoords_{decltype(texcoords_)::allocator_type{mem_resource}},
    face_aabb_{decltype(face_aabb_)::allocator_type{mem_resource}},
    face_code_{decltype(face_code_)::allocator_type{mem_resource}},
    bvh_leaf_node_{decltype(bvh_leaf_node_)::allocator_type{mem_resource}}
{
}

/*!
  \details No detailed description
  */
void Mesh::compileBlas() noexcept
{
  calcAabb();
  calcMortonCode();
  buildBvh();
}

/*!
  \details No detailed description

  \return No description
  */
zisc::pmr::memory_resource* Mesh::resource() const noexcept
{
  return vertices_.get_allocator().resource();
}

/*!
  \details No detailed description

  \param [out] output No description.
  */
void Mesh::writeWavefrontFormat(std::ostream* output) const noexcept
{
  (*output) << "o Mesh" << std::endl;
  for (const F3& vertex : vertices_)
    (*output) << "v " << vertex.x_ << " " << vertex.y_ << " " << vertex.z_ << std::endl;
  for (const F3& normal : normals_)
    (*output) << "vn " << normal.x_ << " " << normal.y_ << " " << normal.z_ << std::endl;
  for (const F2& texcoord : texcoords_)
    (*output) << "vt " << texcoord.x_ << " " << texcoord.y_ << std::endl;
  (*output) << "s 0" << std::endl;
  for (const U3& face : faces_)
    //(*output) << "f " << face.x_ << "/" << face.x_ << "/" << face.x_ << " "
    //                  << face.y_ << "/" << face.y_ << "/" << face.y_ << " "
    //                  << face.z_ << "/" << face.z_ << "/" << face.z_ << std::endl;
    (*output) << "f " << (face.x_ + 1) << " "
                      << (face.y_ + 1) << " "
                      << (face.z_ + 1) << std::endl;
}

/*!
  \details No detailed description
  */
void Mesh::buildBvh() noexcept
{
  //
  const std::size_t lr = faces_.size();
  const std::size_t lc = std::bit_ceil(lr);
  const std::size_t lv = lc - lr;
  const std::size_t clv = lv >> 1;
  const std::size_t clr = (lc >> 1) - clv;
  const std::size_t nr = (2 * clr - 1) + std::popcount(clv);
  const std::size_t max_level = std::bit_width(lc) - 1;

  // Sort
  bvh_leaf_node_.resize(lr);
  std::iota(bvh_leaf_node_.begin(), bvh_leaf_node_.end(), 0);
  const auto cmp = [this](const std::size_t lhs, const std::size_t rhs) noexcept
  {
    return face_code_[lhs] < face_code_[rhs];
  };
  std::sort(bvh_leaf_node_.begin(), bvh_leaf_node_.end(), cmp);

  bvh_info_.max_level_ = max_level;
  bvh_info_.num_of_virtual_leaves_ = lv;
  bvh_node_.resize(lr + nr);
  calcNodeAabb(0, 0);
}

/*!
  \details No detailed description
  */
void Mesh::calcAabb() noexcept
{
  using LimitT = std::numeric_limits<float>;
  const std::array<float, 6> init = {{LimitT::infinity(), LimitT::infinity(), LimitT::infinity(),
                                    -LimitT::infinity(), -LimitT::infinity(), -LimitT::infinity()}};

  const auto extend = [](const std::array<float, 6>& aabb) noexcept
  {
    constexpr float eps = std::numeric_limits<float>::epsilon();
    const std::array<float, 3> size{{aabb[3] - aabb[0],
                                     aabb[4] - aabb[1],
                                     aabb[5] - aabb[2]}};
    std::array<float, 6> new_aabb = aabb;
    new_aabb[0] = (size[0] < eps) ? new_aabb[0] - eps : new_aabb[0];
    new_aabb[1] = (size[1] < eps) ? new_aabb[1] - eps : new_aabb[1];
    new_aabb[2] = (size[2] < eps) ? new_aabb[2] - eps : new_aabb[2];
    new_aabb[3] = (size[0] < eps) ? new_aabb[3] + eps : new_aabb[3];
    new_aabb[4] = (size[1] < eps) ? new_aabb[4] + eps : new_aabb[4];
    new_aabb[5] = (size[2] < eps) ? new_aabb[5] + eps : new_aabb[5];
    return new_aabb;
  };

  zivc::cl::nanairo::Matrix4x4 m{};
  std::copy_n(transformation_.cbegin(), 16, &m.m1_.x);
  local_aabb_ = init;
  world_aabb_ = init;
  for (const F3& vertex : vertices_) {
    // Local AABB calculation
    zivc::cl::float4 v{vertex.x_, vertex.y_, vertex.z_, 1.0f};
    local_aabb_[0] = (std::min)(local_aabb_[0], v.x);
    local_aabb_[1] = (std::min)(local_aabb_[1], v.y);
    local_aabb_[2] = (std::min)(local_aabb_[2], v.z);
    local_aabb_[3] = (std::max)(local_aabb_[3], v.x);
    local_aabb_[4] = (std::max)(local_aabb_[4], v.y);
    local_aabb_[5] = (std::max)(local_aabb_[5], v.z);
    // World AABB calculation
    v = m * v;
    world_aabb_[0] = (std::min)(world_aabb_[0], v.x);
    world_aabb_[1] = (std::min)(world_aabb_[1], v.y);
    world_aabb_[2] = (std::min)(world_aabb_[2], v.z);
    world_aabb_[3] = (std::max)(world_aabb_[3], v.x);
    world_aabb_[4] = (std::max)(world_aabb_[4], v.y);
    world_aabb_[5] = (std::max)(world_aabb_[5], v.z);
  }
  local_aabb_ = extend(local_aabb_);
  world_aabb_ = extend(world_aabb_);

  face_aabb_.clear();
  face_aabb_.reserve(faces_.size());
  for (const U3& face : faces_) {
    std::array<std::size_t, 3> indices = {{face.x_, face.y_, face.z_}};
    std::array<float, 6> aabb = init;
    for (const std::size_t index : indices) {
      const F3& vertex = vertices_[index];
      aabb[0] = (std::min)(aabb[0], vertex.x_);
      aabb[1] = (std::min)(aabb[1], vertex.y_);
      aabb[2] = (std::min)(aabb[2], vertex.z_);
      aabb[3] = (std::max)(aabb[3], vertex.x_);
      aabb[4] = (std::max)(aabb[4], vertex.y_);
      aabb[5] = (std::max)(aabb[5], vertex.z_);
    }
    aabb = extend(aabb);
    face_aabb_.emplace_back(aabb);
  }
}

void Mesh::calcMortonCode() noexcept
{
  const std::array<float, 3> scene_size{{local_aabb_[3] - local_aabb_[0],
                                         local_aabb_[4] - local_aabb_[1],
                                         local_aabb_[5] - local_aabb_[2]}};

  const auto expand = [](std::uint32_t v) noexcept
  {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
  };

  const auto calc_code = [&scene_size, expand](const std::array<float, 3>& v) noexcept
  {
    constexpr auto s = static_cast<float>(1 << 10);
    const auto& size = scene_size;
    const float x = std::clamp((v[0] / size[0]) * s, 0.0f, s - 1.0f);
    const float y = std::clamp((v[1] / size[1]) * s, 0.0f, s - 1.0f);
    const float z = std::clamp((v[2] / size[2]) * s, 0.0f, s - 1.0f);
    const std::uint32_t xx = expand(static_cast<std::uint32_t>(x));
    const std::uint32_t yy = expand(static_cast<std::uint32_t>(y));
    const std::uint32_t zz = expand(static_cast<std::uint32_t>(z));
    return xx * 4 + yy * 2 + zz;
  };

  //
  face_code_.resize(faces_.size());
  for (std::size_t i = 0; i < faces_.size(); ++i) {
    const std::array<float, 6>& aabb = face_aabb_[i];
    const std::array<float, 3> center{{0.5f * (aabb[3] + aabb[0]),
                                       0.5f * (aabb[4] + aabb[1]),
                                       0.5f * (aabb[5] + aabb[2])}};
    const std::uint32_t code = calc_code(center);
    face_code_[i] = code;
  }
}

//void Mesh::calcMortonCode() noexcept
//{
//  std::array<std::size_t, 4> bits{{0, 0, 0, 0}};
//  std::array<std::array<std::size_t, 32>, 4> shifts;
//  // Initialization
//  std::array<float, 3> scene_size{{local_aabb_[3] - local_aabb_[0],
//                                   local_aabb_[4] - local_aabb_[1],
//                                   local_aabb_[5] - local_aabb_[2]}};
//  const std::array<float, 3> scene_box_size = scene_size;
//  std::array<std::uint8_t, 32> axes{};
//  for (std::size_t i = 0; i < 32; ++i) {
//    std::size_t a = 0; 
//    if ((i % 8) == 7) {
//      a = 3;
//    }
//    else {
//      std::size_t largest = (scene_size[0] < scene_size[1]) ? 1 : 0;
//      largest = (scene_size[largest] < scene_size[2]) ? 2 : largest;
//      a = largest;
//      scene_size[a] /= 2.0f;
//    }
//    axes[i] = a;
//    const std::size_t j = bits[a]++;
//    shifts[a][j] = i;
//  }
//  // Compute quantization scales
//  std::array<float, 4> s{{0.0f, 0.0f, 0.0f, 0.0f}};
//  {
//    const float diagonal = std::sqrt(scene_box_size[0] * scene_box_size[0] +
//                                     scene_box_size[1] * scene_box_size[1] +
//                                     scene_box_size[2] * scene_box_size[2]);
//    s[0] = static_cast<float>(1 << bits[0]) / scene_box_size[0];
//    s[1] = static_cast<float>(1 << bits[1]) / scene_box_size[1];
//    s[2] = static_cast<float>(1 << bits[2]) / scene_box_size[2];
//    s[3] = static_cast<float>(1 << bits[3]) / diagonal;
//  }
//
//  const auto expand = [&bits, &shifts](const std::size_t axis, const std::uint32_t x) noexcept
//  {
//    std::uint32_t v = 0,
//                  mask = 1;
//    for (std::size_t i = 0; i < bits[axis]; ++i) {
//      v = v | static_cast<std::uint32_t>((x & mask) << shifts[axis][i]);
//      mask = mask << 1;
//    }
//    return v;
//  };
//
//  //
//  face_code_.resize(faces_.size());
//  for (std::size_t i = 0; i < faces_.size(); ++i) {
//    const std::array<float, 6>& aabb = face_aabb_[i];
//    const std::array<float, 3> tri_size{{aabb[3] - aabb[0], aabb[4] - aabb[1], aabb[5] - aabb[2]}};
//    const float diagonal = std::sqrt(tri_size[0] * tri_size[0] +
//                                     tri_size[1] * tri_size[1] +
//                                     tri_size[2] * tri_size[2]);
//    std::array<std::uint32_t, 4> v{{0, 0, 0, 0}};
//    v[0] = static_cast<std::uint32_t>(0.5f * s[0] * tri_size[0]);
//    v[1] = static_cast<std::uint32_t>(0.5f * s[1] * tri_size[1]);
//    v[2] = static_cast<std::uint32_t>(0.5f * s[2] * tri_size[2]);
//    v[3] = static_cast<std::uint32_t>(s[3] * diagonal);
//
//    const std::uint32_t code = static_cast<std::uint32_t>(expand(0, v[0]) << 3) |
//                               static_cast<std::uint32_t>(expand(1, v[1]) << 2) |
//                               static_cast<std::uint32_t>(expand(2, v[2]) << 1) |
//                               static_cast<std::uint32_t>(expand(3, v[3]));
//    face_code_[i] = code;
//  }
//}

/*!
  \details No detailed description

  \param [in] index No description.
  \param [in] level No description.
  \return No description
  */
std::array<float, 6> Mesh::calcNodeAabb(const std::size_t index, const std::size_t level) noexcept
{
  const std::size_t max_level = bvh_info_.max_level_;
  const std::size_t lvl = (level == 0)
      ? 0
      : bvh_info_.num_of_virtual_leaves_ >> (max_level - (level - 1));
  const std::size_t nvl = 2 * lvl - std::popcount(lvl);

  if (level == max_level) { // leaf node
    const std::size_t i = index - ((1 << level) - 1);
    const std::size_t face_id = bvh_leaf_node_[i];
    const std::array<float, 6>& aabb = face_aabb_[face_id];
    bvh_node_[index - nvl].aabb_ = aabb;
    return aabb;
  }

  const std::size_t clevel = level + 1;
  const std::size_t clvl = bvh_info_.num_of_virtual_leaves_ >> (max_level - clevel);
  const std::size_t cn = (2 << clevel) - 1;

  using LimitT = std::numeric_limits<float>;
  const std::array<float, 6> init = {{LimitT::infinity(), LimitT::infinity(), LimitT::infinity(),
                                    -LimitT::infinity(), -LimitT::infinity(), -LimitT::infinity()}};
  std::array<float, 6> aabb = init;
  if (const std::size_t cindex = (index << 1) + 1; cindex < (cn - clvl)) {
    const std::array<float, 6> caabb = calcNodeAabb(cindex, clevel);
    aabb[0] = (std::min)(aabb[0], caabb[0]);
    aabb[1] = (std::min)(aabb[1], caabb[1]);
    aabb[2] = (std::min)(aabb[2], caabb[2]);
    aabb[3] = (std::max)(aabb[3], caabb[3]);
    aabb[4] = (std::max)(aabb[4], caabb[4]);
    aabb[5] = (std::max)(aabb[5], caabb[5]);
  }
  if (const std::size_t cindex = (index << 1) + 2; cindex < (cn - clvl)) {
    const std::array<float, 6> caabb = calcNodeAabb(cindex, clevel);
    aabb[0] = (std::min)(aabb[0], caabb[0]);
    aabb[1] = (std::min)(aabb[1], caabb[1]);
    aabb[2] = (std::min)(aabb[2], caabb[2]);
    aabb[3] = (std::max)(aabb[3], caabb[3]);
    aabb[4] = (std::max)(aabb[4], caabb[4]);
    aabb[5] = (std::max)(aabb[5], caabb[5]);
  }
  bvh_node_[index - nvl].aabb_ = aabb;
  return aabb;
}

} /* namespace nanairo */
