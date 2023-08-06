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
#include <iostream>
#include <istream>
#include <memory>
#include <string>
#include <vector>
// Tinygltf
#include "tiny_gltf.h"
// Zisc
#include "zisc/binary_serializer.hpp"
#include "zisc/zisc_config.hpp"
#include "zisc/memory/std_memory_resource.hpp"

namespace cmd {

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
GltfScene::GltfScene([[maybe_unused]] zisc::pmr::memory_resource* mem_resource) noexcept
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
  */
void GltfScene::destroy() noexcept
{
}

/*!
  \details No detailed description

  \param [in] data No description.
  */
void GltfScene::load(std::istream& data) noexcept
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
  tinygltf::Model model{};
  tinygltf::TinyGLTF loader{};
  std::string error_message{};
  std::string warning_message{};
  const bool result = loader.LoadBinaryFromMemory(&model,
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
}

} /* namespace cmd */
