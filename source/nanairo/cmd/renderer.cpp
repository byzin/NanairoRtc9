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
#include <algorithm>
#include <array>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string_view>
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
#include "zivc/kernel_set/kernel_set-test_kernel.hpp"
#include "zivc/kernel_set/kernel_set-tone_mapping_kernel.hpp"

namespace nanairo {

namespace kerneldecl {

[[maybe_unused]] auto declPrimarySampleSpaceKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(primary_sample_space_kernel,
                                                                  sampleInPrimarySampleSpaceKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

[[maybe_unused]] auto declTestKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(test_kernel,
                                                                  testKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

[[maybe_unused]] auto declToneMappingKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(tone_mapping_kernel,
                                                                  applyToneMappingKernel,
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
  using TestKernelT = decltype(kerneldecl::declTestKernel());
  using ToneMappingKernelT = decltype(kerneldecl::declToneMappingKernel());


  // Device
  zivc::SharedContext context_;
  zivc::SharedDevice device_;
  // Buffers
  zivc::SharedBuffer<zivc::uint32b> ray_count_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::PrimarySampleSet> sample_set_buffer_;
  zivc::SharedBuffer<zivc::cl::float4> hdr_out_buffer_;
  zivc::SharedBuffer<zivc::cl::uchar4> ldr_out_buffer_;
  zivc::SharedBuffer<zivc::cl::uchar4> ldr_host_buffer_;
  zivc::cl::nanairo::ContextInfo context_info_;
  zivc::cl::nanairo::RenderInfo render_info_;
  // Kenrels
  PrimarySampleSpaceKernelT primary_sample_space_kernel_;
  TestKernelT test_kernel_;
  ToneMappingKernelT tone_mapping_kernel_;

  //!
  void destroy()
  {
    // Kernels
    tone_mapping_kernel_.reset();
    test_kernel_.reset();
    primary_sample_space_kernel_.reset();
    // Buffers
    ldr_host_buffer_.reset();
    ldr_out_buffer_.reset();
    hdr_out_buffer_.reset();
    sample_set_buffer_.reset();
    ray_count_buffer_.reset();
    // Device
    device_.reset();
    context_.reset();
  }

  //!
  void initializeContext(const CliOptions& options, zisc::pmr::memory_resource* mem_resource)
  {
    zivc::ContextOptions context_options{mem_resource};
    context_options.setContextName("Nanairo");
    context_options.setContextVersionMajor(0);
    context_options.setContextVersionMajor(1);
    context_options.setContextVersionPatch(0);
    context_options.enableVulkanBackend(true);
    context_options.enableDebugMode(options.is_debug_mode_);

    context_ = zivc::createContext(context_options);
  }

  //!
  void initializeDevice(const CliOptions& options)
  {
    const std::span device_info_list = context_->deviceInfoList();
    constexpr std::size_t cpu_id = 0;
    constexpr std::size_t gpu0_id = 1;
    constexpr std::size_t gpu1_id = 2;
    std::array<std::size_t, 3> device_id_list{gpu0_id, gpu1_id, cpu_id};
    if (options.is_cpu_forced_)
      device_id_list.fill(cpu_id);
    for (std::size_t i = 0; i < device_id_list.size(); ++i) {
      const std::size_t device_id = device_id_list[i];
      if ((!device_) && (device_id < device_info_list.size())) {
        try {
          device_ = context_->queryDevice(device_id);
        }
        catch (const std::runtime_error& error) {
          std::cerr << "[device" << device_id << "] " << error.what() << std::endl;
        }
      }
    }
    device_->setFenceSize(4);
    device_->setName("NanairoDevice");
  }

  //!
  void initializeRenderContextInfo(const CliOptions& options)
  {
    context_info_.initialize();
    context_info_.setImageResolution({options.image_width_, options.image_height_});
    constexpr zivc::uint16b max_bounce = 4u;
    context_info_.setMaxNumOfBounces(max_bounce);
    constexpr zivc::uint32b seed = 123'456'789u;
    context_info_.setSeed(seed);
  }

  //!
  void initializeKernels([[maybe_unused]] const CliOptions& options)
  {
    // Primary sample space kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                primary_sample_space_kernel,
                                                sampleInPrimarySampleSpaceKernel,
                                                1);
      primary_sample_space_kernel_ = device_->createKernel(params);
      primary_sample_space_kernel_->setName(params.kernelName());
    }
    // Test kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                test_kernel,
                                                testKernel,
                                                1);
      test_kernel_ = device_->createKernel(params);
      test_kernel_->setName(params.kernelName());
    }
    // Tone mapping kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                tone_mapping_kernel,
                                                applyToneMappingKernel,
                                                1);
      tone_mapping_kernel_ = device_->createKernel(params);
      tone_mapping_kernel_->setName(params.kernelName());
    }
  }

  //!
  template <zivc::KernelArg T>
  zivc::SharedBuffer<T> createBuffer(const std::size_t n,
                                     const zivc::BufferInitParams params,
                                     const std::string_view name)
  {
    const zivc::DeviceInfo& info = device_->deviceInfo();
    zivc::BufferInitParams p = params;
    if (info.physicalDeviceType() == zivc::PhysicalDeviceType::kIntegratedGpu)
      p.setUsage(zivc::BufferUsage::kPreferHost);
    zivc::SharedBuffer<T> buffer = device_->createBuffer<T>(p);
    buffer->setSize(n);
    buffer->setName(name);
    return buffer;
  }

  //!
  void initializeBuffers(const CliOptions& options)
  {
    const std::size_t n = options.image_width_ * options.image_height_; // The number of pixels
    ray_count_buffer_ = createBuffer<zivc::uint32b>(1, {zivc::BufferUsage::kPreferDevice, zivc::BufferFlag::kRandomAccessible}, "RayCount");
    const std::size_t set_n = n * zivc::cl::nanairo::calcSampleSetSize(context_info_.maxNumOfBounces());
    sample_set_buffer_ = createBuffer<zivc::cl::nanairo::PrimarySampleSet>(set_n, {zivc::BufferUsage::kPreferDevice}, "PrimarySampleSetBuffer");
    hdr_out_buffer_ = createBuffer<zivc::cl::float4>(n, {zivc::BufferUsage::kPreferDevice}, "HdrOutBuffer");
    ldr_out_buffer_ = createBuffer<zivc::cl::uchar4>(n, {zivc::BufferUsage::kPreferDevice}, "LdrOutBuffer");
    ldr_host_buffer_ = createBuffer<zivc::cl::uchar4>(n, {zivc::BufferUsage::kPreferHost, zivc::BufferFlag::kRandomAccessible}, "LdrHostBuffer");
  }
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
void Renderer::clearFrame()
{
  {
    zivc::BufferLaunchOptions options = data_->hdr_out_buffer_->createOptions();
    options.requestFence(true);
    options.setLabel("ClearHdrBuffer");
    const zivc::cl::float4 value{0.0f, 0.0f, 0.0f, 0.0f};
    zivc::LaunchResult result = zivc::fill(value, data_->hdr_out_buffer_.get(), options);
    result.fence().wait();
  }
}

/*!
  \details No detailed description
  */
void Renderer::destroy() noexcept
{
  if (data_)
    data_->destroy();
  data_.reset();
}

/*!
  \details No detailed description

  \param [out] output No description.
  */
void Renderer::getFrame(LdrImage* output) const
{
  const zivc::cl::uint2 resolution = data_->context_info_.imageResolution();
  const zivc::uint32b num_of_pixels = resolution.x * resolution.y;

  // Run toen mapping kernel
  {
    Data::ToneMappingKernelT& kernel = data_->tone_mapping_kernel_;
    zivc::KernelLaunchOptions options = kernel->createOptions();
    options.setQueueIndex(0);
    options.setWorkSize({{num_of_pixels}});
    options.requestFence(true);
    options.setLabel("ToneMapping");
    zivc::LaunchResult result = kernel->run(*data_->hdr_out_buffer_,
                                            *data_->ldr_out_buffer_,
                                            data_->context_info_,
                                            options);
    result.fence().wait();
  }
  // Data copy
  {
    zivc::BufferLaunchOptions options = data_->ldr_out_buffer_->createOptions();
    options.requestFence(true);
    options.setLabel("CopyLdrBufferToHost");
    zivc::LaunchResult result = zivc::copy(*data_->ldr_out_buffer_,
                                           data_->ldr_host_buffer_.get(),
                                           options);
    result.fence().wait();
  }
  // Writing the result into the LDR image
  {
    const zivc::MappedMemory mem = data_->ldr_host_buffer_->mapMemory();
    std::copy_n(mem.cbegin(), mem.size(), output->data().begin());
  }
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

  // Initialize renderer data
  data_->initializeContext(options, mem_resource_);
  data_->initializeDevice(options);
  data_->initializeRenderContextInfo(options);
  data_->initializeKernels(options);
  data_->initializeBuffers(options);

  //
  if (options.is_debug_mode_)
    printDebugInfo();
}

/*!
  \details No detailed description

  \param [in] frame No description.
  \param [in] iteration No description.
  */
void Renderer::renderFrame(const std::size_t frame,const std::size_t iteration)
{
  data_->render_info_.initialize();
  data_->render_info_.setCurrentFrame(frame);
  data_->render_info_.setCurrentIteration(iteration);
  {
    const zivc::cl::uint2 resolution = data_->context_info_.imageResolution();
    const zivc::uint32b num_of_pixels = resolution.x * resolution.y;
    zivc::MappedMemory mem = data_->ray_count_buffer_->mapMemory();
    mem[0] = num_of_pixels;
  }

  zivc::uint32b n = 0;
  {
    {
      const zivc::MappedMemory mem = data_->ray_count_buffer_->mapMemory();
      n = mem[0];
    }

    // Generate primary sample set
    {
      Data::PrimarySampleSpaceKernelT& kernel = data_->primary_sample_space_kernel_;
      zivc::KernelLaunchOptions options = kernel->createOptions();
      options.setQueueIndex(0);
      options.setWorkSize({{n}});
      options.requestFence(false);
      options.setLabel("PrimarySampleSpace");
      [[maybe_unused]] zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                                               *data_->sample_set_buffer_,
                                                               data_->context_info_,
                                                               data_->render_info_,
                                                               options);
    }

    // Run the test kernel
    {
      Data::TestKernelT& kernel = data_->test_kernel_;
      zivc::KernelLaunchOptions options = kernel->createOptions();
      options.setQueueIndex(0);
      options.setWorkSize({{n}});
      options.requestFence(true);
      options.setLabel("Test");
      zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                              *data_->sample_set_buffer_,
                                              *data_->hdr_out_buffer_,
                                              options);
      result.fence().wait();
    }
  }
}

/*!
  \details No detailed description
  */
void Renderer::printDebugInfo() const noexcept
{
  const auto to_mega_byte = [](const std::size_t bytes) noexcept
  {
    const std::size_t s = 1024 * 1024;
    const double v = static_cast<double>(bytes) / static_cast<double>(s);
    return v;
  };

  const zivc::DeviceInfo& info = data_->device_->deviceInfo();
  std::cout << "  ## render device: " << info.name() << std::endl
            << "      vendor: " << info.vendorName() << std::endl;
  const std::span heap_info_list = info.heapInfoList();
  for (std::size_t i = 0; i < heap_info_list.size(); ++i) {
    const zivc::MemoryHeapInfo& heap_info = heap_info_list[i];
    std::cout << "      heap[" << i << "]" << std::endl
              << "        total: " << std::fixed << std::setprecision(3)
              << to_mega_byte(heap_info.totalSize()) << " MB" << std::endl
              << "        avail: " << std::fixed << std::setprecision(3)
              << to_mega_byte(heap_info.availableSize()) << " MB" << std::endl;
  }
}

} /* namespace nanairo */
