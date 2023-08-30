/*!
  \file bvh-inl.hpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#ifndef NANAIRO_CL_BVH_INL_HPP
#define NANAIRO_CL_BVH_INL_HPP

// Zivc
#include "zivc/cl/bit.hpp"
#include "zivc/cl/limits.hpp"
#include "zivc/cl/types.hpp"
#include "zivc/cl/utility.hpp"
#include "zivc/cl/vector_data.hpp"
// Nanairo
#include "bvh_info.hpp"
#include "bvh_node.hpp"
#include "bvh_node_stack.hpp"
#include "hit_info.hpp"
#include "matrix_4x4.hpp"
#include "ray.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in] bvh_node_buffer No description.
  \param [in] offset No description.
  \return No description
  */
inline
BvhInfo getInfo(zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                const zivc::uint32b offset) noexcept
{
  zivc::ConstGlobalPtr<zivc::uint32b> ptr = bvh_node_buffer + offset;
  const BvhInfo info = zivc::reinterp<zivc::ConstGlobalPtr<BvhInfo>>(ptr)[0];
  return info;
}

/*!
  \details No detailed description

  \param [in] bvh_node_buffer No description.
  \param [in] index No description.
  \param [in] offset No description.
  \return No description
  */
inline
BvhNode getNode(zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                const BvhInfo& info,
                const zivc::uint32b index,
                const zivc::uint32b depth,
                const zivc::uint32b offset) noexcept
{
  constexpr zivc::uint32b info_offset = sizeof(BvhInfo) / sizeof(zivc::uint32b);
  constexpr zivc::uint32b node_offset = sizeof(BvhNode) / sizeof(zivc::uint32b);
  const zivc::uint32b lvl = (depth == 0)
      ? 0
      : info.numOfVirtualLeaves() >> (info.maxDepthLevel() - (depth - 1));
  const zivc::uint32b nvl = 2 * lvl - zivc::popcount(lvl);
  const zivc::uint32b o = offset + info_offset + node_offset * (index - nvl);
  zivc::ConstGlobalPtr<zivc::uint32b> ptr = bvh_node_buffer + o;
  const BvhNode node = zivc::reinterp<zivc::ConstGlobalPtr<BvhNode>>(ptr)[0];
  return node;
}

inline
bool isTlas(const zivc::uint32b offset) noexcept
{
  return offset == 0;
}

inline
bool isLeaf(const BvhInfo& info, const zivc::uint32b depth) noexcept
{
  return info.maxDepthLevel() == depth;
}

inline
zivc::uint32b calcLeafIndex(const zivc::uint32b node_index, const zivc::uint32b depth) noexcept
{
  const zivc::uint32b leaf_index = node_index - ((1 << depth) - 1);
  return leaf_index;
}

inline
void swap(zivc::uint32b& lhs, zivc::uint32b& rhs) noexcept
{
  zivc::uint32b tmp = lhs;
  lhs = rhs;
  rhs = tmp;
}

inline
void swap(float2& lhs, float2& rhs) noexcept
{
  float2 tmp = lhs;
  lhs = rhs;
  rhs = tmp;
}

/*!
  \details No detailed description

  \param [in] node No description.
  \param [in] ray No description.
  \param [in] tmax No description.
  \return No description
  */
inline
float2 testAabbIntersection(const BvhNode& node,
                            const Ray ray,
                            const float tmax) noexcept
{
  const float3 o = ray.origin();
  const float3 d = ray.direction();
  const float3 inv_d = 1.0f / d;

  const float3 p_min = zivc::makeFloat3(node.aabb_min_x_, node.aabb_min_y_, node.aabb_min_z_);
  const float3 p_max = zivc::makeFloat3(node.aabb_max_x_, node.aabb_max_y_, node.aabb_max_z_);

  const float3 d_near = (p_min - o) * inv_d;
  const float3 d_far = (p_max - o) * inv_d;
  const float3 t_near = (zivc::min)(d_near, d_far);
  const float3 t_far = (zivc::max)(d_near, d_far);

  constexpr float k = 1.00000024f;
  const float near = (zivc::max)(t_near.x, (zivc::max)(t_near.y, (zivc::max)(t_near.z, 0.0f)));
  const float far = (zivc::min)(t_far.x, (zivc::min)(t_far.y, (zivc::min)(t_far.z, tmax))) * k;

  return zivc::makeFloat2(near, far);
}

inline
bool isValidNode(const BvhInfo& info,
                 const zivc::uint32b node_index,
                 const zivc::uint32b depth) noexcept
{
  const zivc::uint32b lvl = info.numOfVirtualLeaves() >> (info.maxDepthLevel() - depth);
  const zivc::uint32b cn = (2 << depth) - 1;
  const bool flag = node_index < (cn - lvl);
  return flag;
}

inline
Ray invTransform(const BvhInfo& info, const Ray ray) noexcept
{
  Ray r{};

  const Matrix4x4 m = info.invTransformation();
  {
    float4 o = zivc::castBit<float4>(ray.origin());
    o.w = 1.0f;
    o = m * o;
    r.setOrigin(zivc::castBit<float3>(o));
  }
  {
    float4 d = zivc::castBit<float4>(ray.direction());
    d.w = 0.0f;
    d = m * d;
    r.setDirection(zivc::castBit<float3>(d));
  }

  return r;
}

inline
void getTriangle(zivc::ConstGlobalPtr<zivc::uint32b> face_buffer,
                 zivc::ConstGlobalPtr<float> geometry_buffer,
                 const BvhInfo& info,
                 const zivc::uint32b face_index,
                 zivc::PrivatePtr<float3> v0,
                 zivc::PrivatePtr<float3> v1,
                 zivc::PrivatePtr<float3> v2) noexcept
{
  const zivc::uint32b face_offset = info.faceOffset();
  const uint3 face = zivc::vload3(face_index, face_buffer + face_offset);
  const zivc::uint32b geom_offset = info.geometryOffset();
  v0[0] = zivc::vload3(face.x, geometry_buffer + geom_offset);
  v1[0] = zivc::vload3(face.y, geometry_buffer + geom_offset);
  v2[0] = zivc::vload3(face.z, geometry_buffer + geom_offset);
}

inline
float4 testTriangleIntersection(const float3 v0,
                                const float3 v1,
                                const float3 v2,
                                const Ray ray,
                                const float tmax) noexcept
{
  using FLimitT = zivc::NumericLimits<float>;

  const float3 o = ray.origin();
  const float3 d = ray.direction();

  const float3 e1 = v1 - v0;
  const float3 e2 = v2 - v0;

  const float3 r = o - v0;
  const float3 p1 = zivc::cross(r, e1);
  const float3 p2 = zivc::cross(d, e2);

  const float k = zivc::dot(p2, e1);
  const float t = zivc::dot(p1, e2) / k;
  const float u = zivc::dot(p2, r) / k;
  const float v = zivc::dot(p1, d) / k;

  float4 result = zivc::makeFloat4(0.0f, 0.0f, 0.0f, FLimitT::infinity());
  const bool has_hit = (0.0f <= u) && (0.0f <= v) && (u + v <= 1.0f) && (0.0f < t) && (t < tmax);
  if (has_hit) {
    const float3 normal = zivc::cross(e1, e2);
    result = zivc::castBit<float4>(zivc::normalize(normal));
    result.w = t;
  }
  return result;
}

//inline
//float4 testTriangleIntersection(const float3 v0,
//                                const float3 v1,
//                                const float3 v2,
//                                const Ray ray,
//                                const float tmax) noexcept
//{
//  using FLimitT = zivc::NumericLimits<float>;
//
//  const float3 o = ray.origin();
//  const float3 d = ray.direction();
//
//  const float3 pos0 = v0 - o;
//  const float3 pos1 = v1 - o;
//  const float3 pos2 = v2 - o;
//  const float3 edge0 = v2 - v0;
//  const float3 edge1 = v0 - v1;
//  const float3 edge2 = v1 - v2;
//  const float3 normal = zivc::cross(edge1, edge0);
//
//  const float u = zivc::dot(zivc::cross(pos0 + pos2, edge0), d);
//  const float v = zivc::dot(zivc::cross(pos1 + pos0, edge1), d);
//  const float w = zivc::dot(zivc::cross(pos2 + pos1, edge2), d);
//  const float t = zivc::dot(pos0, normal) * 2.0f;
//
//  const bool has_hit = (0.0f <= u) && (0.0f <= v) && (0.0f <= w) && (0.0f < t) && (t < tmax);
//  float4 n = zivc::castBit<float4>(normal);
//  n.w = has_hit ? t : FLimitT::infinity();
//  return n;
//}

/*!
  \details No detailed description

  \param [in] geometry_buffer No description.
  \param [in] face_buffer No description.
  \param [in] bvh_node_buffer No description.
  \param [in] bvh_map_buffer No description.
  \param [in] node_stack No description.
  \param [in] ray No description.
  \return No description
  */
inline
HitInfo castRay(zivc::ConstGlobalPtr<zivc::uint32b> face_buffer,
                zivc::ConstGlobalPtr<float> geometry_buffer,
                zivc::ConstGlobalPtr<zivc::uint32b> bvh_node_buffer,
                zivc::ConstGlobalPtr<zivc::uint32b> bvh_map_buffer,
                zivc::LocalPtr<BvhNodeStack> node_stack,
                const Ray ray) noexcept
{
  using FLimitT = zivc::NumericLimits<float>;

  HitInfo hit_info{};
  hit_info.initialize();

  zivc::uint32b node_index = 0;
  zivc::uint32b depth = 0;
  zivc::uint32b offset = 0;
  zivc::uint32b stack_index = 0;
  Ray r = ray;
  BvhInfo bvh_info = getInfo(bvh_node_buffer, offset);
  const zivc::uint32b depth_offset = bvh_info.maxDepthLevel();
  while (true) {
    if (isLeaf(bvh_info, depth)) {
      const zivc::uint32b leaf_index = calcLeafIndex(node_index, depth);
      if (isTlas(offset)) {
        // Go into the blas
        offset = bvh_map_buffer[leaf_index];
        node_index = 0;
        depth = 0;
        bvh_info = getInfo(bvh_node_buffer, offset);
        r = invTransform(bvh_info, r);
        continue;
      }
      else { // BLAS case
        float3 v0 = zivc::makeFloat3(0.0f, 0.0f, 0.0f),
               v1 = zivc::makeFloat3(0.0f, 0.0f, 0.0f),
               v2 = zivc::makeFloat3(0.0f, 0.0f, 0.0f);
        getTriangle(face_buffer, geometry_buffer, bvh_info, leaf_index, &v0, &v1, &v2);
        const float4 result = testTriangleIntersection(v0, v1, v2, r, hit_info.distance());
        const bool has_hit = result.w != FLimitT::infinity();
        if (has_hit) {
          hit_info.setDistance(result.w);
          float4 normal = result;
          normal.w = 0.0f;
          normal = bvh_info.transformation() * normal;
          normal = zivc::normalize3(normal);
          hit_info.setGeometryNormal(zivc::castBit<float3>(normal));
        }
      }
    }
    else { // Internal node case
      zivc::uint32b cindexl = (node_index << 1) + 1;
      zivc::uint32b cindexr = (node_index << 1) + 2;
      float2 tl = zivc::makeFloat2(FLimitT::infinity(), -FLimitT::infinity());
      float2 tr = zivc::makeFloat2(FLimitT::infinity(), -FLimitT::infinity());
      if (isValidNode(bvh_info, cindexl, depth + 1)) {
        const BvhNode node = getNode(bvh_node_buffer, bvh_info, cindexl, depth + 1, offset);
        tl = testAabbIntersection(node, r, hit_info.distance());
      }
      if (isValidNode(bvh_info, cindexr, depth + 1)) {
        const BvhNode node = getNode(bvh_node_buffer, bvh_info, cindexr, depth + 1, offset);
        tr = testAabbIntersection(node, r, hit_info.distance());
      }
      const bool has_hit_l = tl.x <= tl.y;
      const bool has_hit_r = tr.x <= tr.y;
      if ((!has_hit_l && has_hit_r) || (has_hit_l && has_hit_r && (tr.x < tl.x))) {
        swap(cindexl, cindexr);
      }
      if (has_hit_l && has_hit_r) {
        node_stack[0].stack_[stack_index++] = zivc::makeUInt2(cindexr, depth_offset + depth + 1);
      }
      if (has_hit_l || has_hit_r) {
        node_index = cindexl;
        ++depth;
        continue;
      }
    }

    if (0 < stack_index) {
      // Pop next node index from the stack
      const uint2 data = node_stack[0].stack_[--stack_index];
      const bool is_back_to_tlas = !isTlas(offset) && (data.y <= depth_offset);
      node_index = data.x;
      depth = is_back_to_tlas ? data.y : data.y - depth_offset;
      if (is_back_to_tlas) {
        offset = 0;
        bvh_info = getInfo(bvh_node_buffer, offset);
        r = ray;
      }
    }
    else {
      // Finish the traverse
      break;
    }
  }

  return hit_info;
}

} /* namespace nanairo */

#endif /* NANAIRO_CL_BVH_INL_HPP */
