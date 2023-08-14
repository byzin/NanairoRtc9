/*!
  \file renderer.cpp
  \author Sho Ikeda
  \brief No brief description

  \details
  No detailed description.

  \copyright
  Copyright (c) 2015-2023 Sho Ikeda
  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
  */

#include "renderer.hpp"
// Standard C++ library
#include <array>
#include <cstddef>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
// Zisc
#include "zisc/memory/std_memory_resource.hpp"
// Zivc
#include "zivc/zivc.hpp"
// Nanairo
#include "cli.hpp"
#include "gltf_scene.hpp"
#include "ldr_image.hpp"
// Nanairo kenrel
#include "zivc/kernel_set/kernel_set-primary_sample_space_kernel.hpp"

namespace nanairo {

namespace kerneldecl {

[[maybe_unused]] auto declPrimarySampleSpaceKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(primary_sample_space_kernel,
                                                                  sampleInPrimarySampleSpaceKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

} /* namespace kerneldecl */


/*!
  No detailed description.
  */
struct Renderer::Data
{
  // Kernel declaration
  using PrimarySampleSpaceKernelT = decltype(kerneldecl::declPrimarySampleSpaceKernel());


  // Device
  zivc::SharedContext context_;
  zivc::SharedDevice device_;
  // Buffers
  // Kenrels
  PrimarySampleSpaceKernelT primary_sample_space_kernel_;
};

/*!
  \details No detailed description

  \param [in,out] mem_resource No description.
  */
Renderer::Renderer(zisc::pmr::memory_resource* mem_resource) noexcept : mem_resource_{mem_resource}
{
}

/*!
  \details No detailed description
  */
Renderer::~Renderer() noexcept
{
  destroy();
}

/*!
  \details No detailed description
  */
void Renderer::destroy() noexcept
{
  if (data_) {
    // Kernels
    data_->primary_sample_space_kernel_.reset();
    // Device
    data_->device_.reset();
    data_->context_.reset();
  }
  data_.reset();
}

/*!
  \details No detailed description

  \param [in] options No description.
  \param [in] scene No description.
  */
void Renderer::initialize(const CliOptions& options, const GltfScene& scene)
{
  destroy();

  const zisc::pmr::polymorphic_allocator<Data> alloc{mem_resource_};
  data_ = std::allocate_shared<Data>(alloc);

  zivc::ContextOptions context_options{mem_resource_};
  context_options.setContextName("Nanairo");
  context_options.setContextVersionMajor(0);
  context_options.setContextVersionMajor(1);
  context_options.setContextVersionPatch(0);
  context_options.enableVulkanBackend(true);
  context_options.enableDebugMode(options.is_debug_mode_);

  // Create a zivc context
  data_->context_ = zivc::createContext(context_options);

  // Create a device
  const std::span device_info_list = data_->context_->deviceInfoList();
  constexpr std::size_t cpu_id = 0;
  constexpr std::size_t gpu0_id = 1;
  constexpr std::size_t gpu1_id = 2;
  std::array<std::size_t, 3> device_id_list{gpu0_id, gpu1_id, cpu_id};
  if (options.is_cpu_forced_)
    device_id_list.fill(cpu_id);
  for (std::size_t i = 0; i < device_id_list.size(); ++i) {
    if ((!data_->device_) && (i < device_info_list.size())) {
      try {
        data_->device_ = data_->context_->queryDevice(i);
      }
      catch (const std::runtime_error& error) {
        std::cerr << "[device" << i << "] " << error.what() << std::endl;
      }
    }
  }
}

/*!
  \details No detailed description

  \param [in] frame No description.
  \param [out] output No description.
  */
void Renderer::renderFrame(const std::size_t frame, LdrImage* output) noexcept
{
}

} /* namespace nanairo */
