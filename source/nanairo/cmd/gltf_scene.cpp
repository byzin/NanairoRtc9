/*!
  \file gltf_scene.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "gltf_scene.hpp"
// Standard C++ library
#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <istream>
#include <limits>
#include <memory>
#include <numeric>
#include <span>
#include <string>
#include <vector>
// Tinygltf
#include "tiny_gltf.h"
// Zisc
#include "zisc/binary_serializer.hpp"
#include "zisc/bit.hpp"
#include "zisc/function_reference.hpp"
#include "zisc/zisc_config.hpp"
#include "zisc/memory/std_memory_resource.hpp"
// Nanairo
#include "camera.hpp"
#include "mesh.hpp"
#include "zivc/kernel_set/kernel_set-ray_cast_kernel.hpp"

namespace nanairo {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
GltfScene::GltfScene([[maybe_unused]] zisc::pmr::memory_resource* mem_resource) noexcept :
    meshes_{decltype(meshes_)::allocator_type{mem_resource}},
    mesh_code_{decltype(mesh_code_)::allocator_type{mem_resource}},
    bvh_node_{decltype(bvh_node_)::allocator_type{mem_resource}},
    bvh_leaf_node_{decltype(bvh_leaf_node_)::allocator_type{mem_resource}}
{
}

/*!
  \details No detailed description
  */
GltfScene::~GltfScene() noexcept
{
  destroy();
}

/*!
  \details No detailed description

  \return No description
  */
const Camera& GltfScene::camera() const noexcept
{
  return camera_;
}

/*!
  \details No detailed description
  */
void GltfScene::destroy() noexcept
{
}

/*!
  \details No detailed description

  \param [in] data No description.
  */
void GltfScene::loadBinary(std::istream& data) noexcept
{
  using zisc::uint8b;

  // 
  std::vector<uint8b> scene_data{};
  {
    const std::streamsize size = zisc::BinarySerializer::getDistance(&data);
    scene_data.resize(size);
    zisc::BinarySerializer::read(scene_data.data(), &data, size);
  }

  //
  tinygltf::TinyGLTF loader{};
  std::string error_message{};
  std::string warning_message{};
  const bool result = loader.LoadBinaryFromMemory(&model_,
                                                  &error_message,
                                                  &warning_message,
                                                  scene_data.data(),
                                                  scene_data.size());
  if (!warning_message.empty()) {
    std::cout << warning_message << std::endl;
  }
  if (!error_message.empty()) {
    std::cerr << error_message << std::endl;
  }
  if (!result) {
    std::cerr << "  Tinygltf: Failed to parse glTF." << std::endl;
  }

  compileScene();
}

/*!
  \details No detailed description

  \return No description
  */
std::span<const Mesh> GltfScene::meshList() const noexcept
{
  return meshes_;
}

/*!
  \details No detailed description

  \return No description
  */
const tinygltf::Model& GltfScene::model() const noexcept
{
  return model_;
}

/*!
  \details No detailed description
  */
void GltfScene::buildBvh() noexcept
{
  //
  const std::size_t lr = meshes_.size();
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
    return mesh_code_[lhs] < mesh_code_[rhs];
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
void GltfScene::calcAabb() noexcept
{
  using LimitT = std::numeric_limits<float>;
  const std::array<float, 6> init = {{LimitT::infinity(), LimitT::infinity(), LimitT::infinity(),
                                    -LimitT::infinity(), -LimitT::infinity(), -LimitT::infinity()}};

  aabb_ = init;
  for (const Mesh& mesh : meshes_) {
    aabb_[0] = (std::min)(aabb_[0], mesh.world_aabb_[0]);
    aabb_[1] = (std::min)(aabb_[1], mesh.world_aabb_[1]);
    aabb_[2] = (std::min)(aabb_[2], mesh.world_aabb_[2]);
    aabb_[3] = (std::max)(aabb_[3], mesh.world_aabb_[3]);
    aabb_[4] = (std::max)(aabb_[4], mesh.world_aabb_[4]);
    aabb_[5] = (std::max)(aabb_[5], mesh.world_aabb_[5]);
  }
}

/*!
  \details No detailed description
  */
void GltfScene::calcMortonCode() noexcept
{
  std::array<std::size_t, 4> bits{{0, 0, 0, 0}};
  std::array<std::array<std::size_t, 32>, 4> shifts;
  // Initialization
  std::array<float, 3> scene_size{{aabb_[3] - aabb_[0], aabb_[4] - aabb_[1], aabb_[5] - aabb_[2]}};
  const std::array<float, 3> scene_box_size = scene_size;
  std::array<std::uint8_t, 32> axes{};
  for (std::size_t i = 0; i < 32; ++i) {
    std::size_t a = 0; 
    if ((i % 8) == 7) {
      a = 3;
    }
    else {
      std::size_t largest = (scene_size[0] < scene_size[1]) ? 1 : 0;
      largest = (scene_size[largest] < scene_size[2]) ? 2 : largest;
      a = largest;
      scene_size[a] /= 2.0f;
    }
    axes[i] = a;
    const std::size_t j = bits[a]++;
    shifts[a][j] = i;
  }
  // Compute quantization scales
  std::array<float, 4> s{{0.0f, 0.0f, 0.0f, 0.0f}};
  {
    const float diagonal = std::sqrt(scene_box_size[0] * scene_box_size[0] +
                                     scene_box_size[1] * scene_box_size[1] +
                                     scene_box_size[2] * scene_box_size[2]);
    s[0] = static_cast<float>(1 << bits[0]) / scene_box_size[0];
    s[1] = static_cast<float>(1 << bits[1]) / scene_box_size[1];
    s[2] = static_cast<float>(1 << bits[2]) / scene_box_size[2];
    s[3] = static_cast<float>(1 << bits[3]) / diagonal;
  }

  const auto expand = [&bits, &shifts](const std::size_t axis, const std::uint32_t x) noexcept
  {
    std::uint32_t v = 0,
                  mask = 1;
    for (std::size_t i = 0; i < bits[axis]; ++i) {
      v = v | static_cast<std::uint32_t>((x & mask) << shifts[axis][i]);
      mask = mask << 1;
    }
    return v;
  };

  //
  mesh_code_.resize(meshes_.size());
  for (std::size_t i = 0; i < meshes_.size(); ++i) {
    const std::array<float, 6>& aabb = meshes_[i].world_aabb_;
    const std::array<float, 3> tri_size{{aabb[3] - aabb[0], aabb[4] - aabb[1], aabb[5] - aabb[2]}};
    const float diagonal = std::sqrt(tri_size[0] * tri_size[0] +
                                     tri_size[1] * tri_size[1] +
                                     tri_size[2] * tri_size[2]);
    std::array<std::uint32_t, 4> v{{0, 0, 0, 0}};
    v[0] = static_cast<std::uint32_t>(0.5f * s[0] * tri_size[0]);
    v[1] = static_cast<std::uint32_t>(0.5f * s[1] * tri_size[1]);
    v[2] = static_cast<std::uint32_t>(0.5f * s[2] * tri_size[2]);
    v[3] = static_cast<std::uint32_t>(s[3] * diagonal);

    const std::uint32_t code = static_cast<std::uint32_t>(expand(0, v[0]) << 3) |
                               static_cast<std::uint32_t>(expand(1, v[1]) << 2) |
                               static_cast<std::uint32_t>(expand(2, v[2]) << 1) |
                               static_cast<std::uint32_t>(expand(3, v[3]));
    mesh_code_[i] = code;
  }
}

/*!
  \details No detailed description

  \param [in] index No description.
  \param [in] level No description.
  \return No description
  */
std::array<float, 6> GltfScene::calcNodeAabb(const std::size_t index, const std::size_t level) noexcept
{
  const std::size_t max_level = bvh_info_.max_level_;
  const std::size_t lvl = (level == 0)
      ? 0
      : bvh_info_.num_of_virtual_leaves_ >> (max_level - (level - 1));
  const std::size_t nvl = 2 * lvl - std::popcount(lvl);

  if (level == max_level) { // leaf node
    const std::size_t i = index - ((1 << level) - 1);
    const std::size_t face_id = bvh_leaf_node_[i];
    const std::array<float, 6>& aabb = meshes_[face_id].world_aabb_;
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

/*!
  \details No detailed description
  */
void GltfScene::compileTlas() noexcept
{
  calcAabb();
  calcMortonCode();
  buildBvh();
}

/*!
  \details No detailed description
  */
void GltfScene::compileGeometries() noexcept
{
  const tinygltf::Scene& s = model().scenes[model().defaultScene];
  for (const int index : s.nodes) {
    const Matrix4x4 m = Matrix4x4::identity();
    processNode(index, m, m);
  }
}

void GltfScene::compileScene() noexcept
{
  // Clear data
  meshes_.clear();

  //
  constexpr std::size_t default_mesh_size = 1024;
  meshes_.reserve(default_mesh_size);

  // Compile the scene
  compileGeometries();

  //
  //printDebugInfo();

  // Compile meshes
  for (Mesh& mesh : meshes_)
    mesh.compileBlas();
  compileTlas();
}

/*!
  \details No detailed description

  \param [in] ptr No description.
  \return No description
  */
template <std::integral Integer>
std::uint32_t GltfScene::getIndexImpl(const unsigned char* const ptr) noexcept
{
  const Integer index = *reinterpret_cast<const Integer* const>(ptr);
  return static_cast<std::uint32_t>(index);
}

auto GltfScene::getIndexGetter(const int component_type) noexcept -> IndexGetterFuncT
{
  IndexGetterFuncT func{};
  switch (component_type) {
   case TINYGLTF_COMPONENT_TYPE_BYTE: {
    func = &getIndexImpl<std::int8_t>;
    break;
   }
   case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
    func = &getIndexImpl<std::uint8_t>;
    break;
   }
   case TINYGLTF_COMPONENT_TYPE_SHORT: {
    func = &getIndexImpl<std::int16_t>;
    break;
   }
   case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
    func = &getIndexImpl<std::uint16_t>;
    break;
   }
   case TINYGLTF_COMPONENT_TYPE_INT: {
    func = &getIndexImpl<std::int32_t>;
    break;
   }
   case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
    func = &getIndexImpl<std::uint32_t>;
    break;
   }
   default: {
    std::cerr << "[error] Unsupported index type: " << component_type << std::endl;
    break;
   }
  }

  return func;
}

/*!
  \details No detailed description

  \return No description
  */
zisc::pmr::memory_resource* GltfScene::resource() const noexcept
{
  return meshes_.get_allocator().resource();
}

/*!
  \details No detailed description
  */
void GltfScene::printDebugInfo() const noexcept
{
  for (std::size_t i = 0; i < meshes_.size(); ++i) {
    //
    constexpr std::size_t max_name_length = 32;
    std::array<char, max_name_length> name{};
    std::snprintf(name.data(), name.size(), "mesh%03d.obj", static_cast<int>(i));
    std::ofstream obj_file{name.data()};
    //
    const Mesh& mesh = meshes_[i];
    mesh.writeWavefrontFormat(&obj_file);
  }
}

/*!
  \details No detailed description

  \param [in] index No description.
  \param [in] transformation No description.
  */
void GltfScene::processCamera(const std::size_t index,
                              const Matrix4x4& transformation) noexcept
{
  const tinygltf::Camera& camera = model().cameras[index];
  Camera& dest = camera_;

  dest.yfov_ = camera.perspective.yfov;
  dest.transformation_ = zisc::bit_cast<decltype(dest.transformation_)>(transformation);
}

/*!
  \details No detailed description

  \param [in] index No description.
  \param [in] transformation No description.
  \param [in] inv_transformation No description.
  */
void GltfScene::processMesh(const std::size_t index,
                            const Matrix4x4& transformation,
                            const Matrix4x4& inv_transformation) noexcept
{
  const tinygltf::Mesh& mesh = model().meshes[index];
  Mesh& dest = meshes_.emplace_back(resource());
  dest.transformation_ = zisc::bit_cast<std::array<float, 16>>(transformation);
  dest.inv_transformation_ = zisc::bit_cast<std::array<float, 16>>(inv_transformation);

  // Reserve the memory for the mesh first
  std::size_t num_of_faces = 0;
  std::size_t num_of_vertices = 0;
  std::size_t num_of_normals = 0;
  std::size_t num_of_texcoords = 0;
  for (std::size_t i = 0; i < mesh.primitives.size(); ++i) {
    const tinygltf::Primitive& prim = mesh.primitives[i];

    // Calculate the size of faces
    {
      const tinygltf::Accessor& accessor = model().accessors[prim.indices];
      num_of_faces += accessor.count;
    }

    // Calculate the size of vertices, normals and texcoords
    switch (prim.mode) {
     case TINYGLTF_MODE_TRIANGLE_FAN: {
      std::cerr << "## Triangle Fan." << std::endl;
      break;
     }
     case TINYGLTF_MODE_TRIANGLE_STRIP: {
      std::cerr << "## Triangle Strip." << std::endl;
      break;
     }
     case TINYGLTF_MODE_TRIANGLES: {
      using PairT = decltype(prim.attributes)::value_type;
      for (const PairT& attribute : prim.attributes) {
        const tinygltf::Accessor& accessor = model().accessors[attribute.second];
        if (attribute.first == "POSITION")
          num_of_vertices += accessor.count;
        if (attribute.first == "NORMAL")
          num_of_normals += accessor.count;
        if (attribute.first == "TEXCOORD_0")
          num_of_texcoords += accessor.count;
      }
      break;
     }
     default: {
      std::cerr << "[error] Unsupported primitive mode: " << prim.mode << std::endl;
      break;
     }
    }
  }
  dest.faces_.reserve(num_of_faces);
  dest.vertices_.reserve(num_of_vertices);
  dest.normals_.reserve(num_of_normals);
  dest.texcoords_.reserve(num_of_texcoords);

  // Load the actual mesh info
  for (std::size_t i = 0; i < mesh.primitives.size(); ++i) {
    const tinygltf::Primitive& prim = mesh.primitives[i];

    // Load faces
    {
      const tinygltf::Accessor& accessor = model().accessors[prim.indices];
      const tinygltf::BufferView& buffer_view = model().bufferViews[accessor.bufferView];
      const tinygltf::Buffer& buffer = model().buffers[buffer_view.buffer];
      const unsigned char* const ptr = buffer.data.data() + (buffer_view.byteOffset + accessor.byteOffset);
      const auto byte_stride = static_cast<std::size_t>(accessor.ByteStride(buffer_view));
      const std::size_t offset = dest.vertices_.size();
      const zisc::FunctionReference get_index = getIndexGetter(accessor.componentType);
      for (std::size_t j = 0; j < accessor.count; j += 3) {
        const std::uint32_t i0 = offset + get_index(ptr + byte_stride * (j + 0));
        const std::uint32_t i1 = offset + get_index(ptr + byte_stride * (j + 1));
        const std::uint32_t i2 = offset + get_index(ptr + byte_stride * (j + 2));
        dest.faces_.emplace_back(i0, i1, i2);
      }
    }

    // Load vertices, normals and texcoords
    switch (prim.mode) {
     case TINYGLTF_MODE_TRIANGLE_FAN: {
      std::cerr << "## Triangle Fan." << std::endl;
      break;
     }
     case TINYGLTF_MODE_TRIANGLE_STRIP: {
      std::cerr << "## Triangle Strip." << std::endl;
      break;
     }
     case TINYGLTF_MODE_TRIANGLES: {
      using PairT = decltype(prim.attributes)::value_type;
      for (const PairT& attribute : prim.attributes) {
        const tinygltf::Accessor& accessor = model().accessors[attribute.second];
        const tinygltf::BufferView& buffer_view = model().bufferViews[accessor.bufferView];
        const tinygltf::Buffer& buffer = model().buffers[buffer_view.buffer];
        const unsigned char* const ptr = buffer.data.data() + (buffer_view.byteOffset + accessor.byteOffset);
        const auto byte_stride = static_cast<std::size_t>(accessor.ByteStride(buffer_view));
        if (attribute.first == "POSITION") {
          if ((accessor.type == TINYGLTF_TYPE_VEC3) &&
              (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)) {
            for (std::size_t j = 0; j < accessor.count; ++j) {
              const unsigned char* const p = ptr + j * byte_stride;
              const std::span<const float> s{reinterpret_cast<const float*>(p), 3};
              dest.vertices_.emplace_back(s[0], s[1], s[2]);
            }
          }
          else {
            std::cerr << "[error] A position has unsupported component type." << std::endl;
          }
        }
        //if (attribute.first == "NORMAL") {
        //  if ((accessor.type == TINYGLTF_TYPE_VEC3) &&
        //      (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)) {
        //    for (std::size_t j = 0; j < accessor.count; ++j) {
        //      const unsigned char* const p = ptr + j * byte_stride;
        //      const std::span<const float> s{reinterpret_cast<const float*>(p), 3};
        //      dest.normals_.emplace_back(s[0], s[1], s[2]);
        //    }
        //  }
        //  else {
        //    std::cerr << "[error] A normal has unsupported component type." << std::endl;
        //  }
        //}
        //if (attribute.first == "TEXCOORD_0") {
        //  if ((accessor.type == TINYGLTF_TYPE_VEC2) &&
        //      (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT)) {
        //    for (std::size_t j = 0; j < accessor.count; ++j) {
        //      const unsigned char* const p = ptr + j * byte_stride;
        //      const std::span<const float> s{reinterpret_cast<const float*>(p), 2};
        //      dest.texcoords_.emplace_back(s[0], s[1]);
        //    }
        //  }
        //  else {
        //    std::cerr << "[error] A texcoord has unsupported component type." << std::endl;
        //  }
        //}
      }
      break;
     }
     default: {
      std::cerr << "[error] Unsupported primitive mode: " << prim.mode << std::endl;
      break;
     }
    }
  }
}

/*!
  \details No detailed description

  \param [in] index No description.
  \param [in] parent_transformation No description.
  \param [in] parent_inv_transformation No description.
  \param [in] level No description.
  */
void GltfScene::processNode(const std::size_t index,
                            const Matrix4x4& parent_transformation,
                            const Matrix4x4& parent_inv_transformation,
                            const std::size_t level) noexcept
{
  const tinygltf::Node& node = model().nodes[index];

//  std::cout << "## node[" << level << "] =" << node.name << std::endl;

  Matrix4x4 transformation = parent_transformation;
  Matrix4x4 inv_transformation = parent_inv_transformation;

  if (!node.translation.empty()) {
    const auto tx = static_cast<float>(node.translation[0]);
    const auto ty = static_cast<float>(node.translation[1]);
    const auto tz = static_cast<float>(node.translation[2]);
    {
      const Matrix4x4 m = zivc::cl::nanairo::getTranslationMatrix(tx, ty, tz);
      transformation = transformation * m;
    }
    {
      const Matrix4x4 m = zivc::cl::nanairo::getInvTranslationMatrix(tx, ty, tz);
      inv_transformation = m * inv_transformation;
    }
  }

  if (!node.rotation.empty()) {
    const auto rx = static_cast<float>(node.rotation[0]);
    const auto ry = static_cast<float>(node.rotation[1]);
    const auto rz = static_cast<float>(node.rotation[2]);
    const auto rw = static_cast<float>(node.rotation[3]);
    const zivc::cl::float4 r = zivc::cl::zivc::normalize(zivc::cl::float4{rx, ry, rz, rw});
    const zivc::cl::nanairo::Quaternion q = {r};
    {
      const Matrix4x4 m = zivc::cl::nanairo::getRotationMatrix(q);
      transformation = transformation * m;
    }
    {
      const Matrix4x4 m = zivc::cl::nanairo::getInvRotationMatrix(q);
      inv_transformation = m * inv_transformation;
    }
  }

  if (!node.scale.empty()) {
    const auto sx = static_cast<float>(node.scale[0]);
    const auto sy = static_cast<float>(node.scale[1]);
    const auto sz = static_cast<float>(node.scale[2]);
    {
      const Matrix4x4 m = zivc::cl::nanairo::getScalingMatrix(sx, sy, sz);
      transformation = transformation * m;
    }
    {
      const Matrix4x4 m = zivc::cl::nanairo::getInvScalingMatrix(sx, sy, sz);
      inv_transformation = m * inv_transformation;
    }
  }

  if (!node.matrix.empty()) {
    std::cerr << "[error] node matrix isn't supported yet." << std::endl;
  }

  if (node.mesh != -1) {
    const auto mesh_index = static_cast<std::size_t>(node.mesh);
    processMesh(mesh_index, transformation, inv_transformation);
  }

//  if (node.light != -1) {
//    std::cout << "##   light" << std::endl;
//  }

  if (node.camera != -1) {
    const auto camera_index = static_cast<std::size_t>(node.camera);
    processCamera(camera_index, transformation);
  }

//  if (!node.extensions.empty()) {
//    std::cout << "##   extensions" << std::endl;
//  }

  for (const int child_index : node.children) {
    processNode(child_index, transformation, inv_transformation, level + 1);
  }
}

} /* namespace nanairo */
