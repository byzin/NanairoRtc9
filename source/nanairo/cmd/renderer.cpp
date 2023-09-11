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
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string_view>
// Zisc
#include "zisc/bit.hpp"
#include "zisc/memory/std_memory_resource.hpp"
// Zivc
#include "zivc/zivc.hpp"
// Nanairo
#include "cli.hpp"
#include "gltf_scene.hpp"
#include "ldr_image.hpp"
// Nanairo kenrel
#include "zivc/kernel_set/kernel_set-primary_sample_space_kernel.hpp"
#include "zivc/kernel_set/kernel_set-ray_cast_kernel.hpp"
#include "zivc/kernel_set/kernel_set-feature_ray_cast_kernel.hpp"
#include "zivc/kernel_set/kernel_set-ray_generation_kernel.hpp"
#include "zivc/kernel_set/kernel_set-feature_ray_generation_kernel.hpp"
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

[[maybe_unused]] auto declRayGenerationKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(ray_generation_kernel,
                                                                  generateRayKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

[[maybe_unused]] auto declFeatureRayGenerationKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(feature_ray_generation_kernel,
                                                                  generateFeatureRayKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

[[maybe_unused]] auto declRayCastKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(ray_cast_kernel,
                                                                  castRayKernel,
                                                                  1);
  return zivc::SharedDevice{}->createKernel(p);
}

[[maybe_unused]] auto declFeatureRayCastKernel()
{
  const zivc::KernelInitParams p = ZIVC_CREATE_KERNEL_INIT_PARAMS(feature_ray_cast_kernel,
                                                                  castFeatureRayKernel,
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
  using RayGenerationKernelT = decltype(kerneldecl::declRayGenerationKernel());
  using FeatureRayGenerationKernelT = decltype(kerneldecl::declFeatureRayGenerationKernel());
  using RayCastKernelT = decltype(kerneldecl::declRayCastKernel());
  using FeatureRayCastKernelT = decltype(kerneldecl::declFeatureRayCastKernel());
  using TestKernelT = decltype(kerneldecl::declTestKernel());
  using ToneMappingKernelT = decltype(kerneldecl::declToneMappingKernel());


  // Device
  zivc::SharedContext context_;
  zivc::SharedDevice device_;
  // Buffers
  zivc::SharedBuffer<zivc::uint32b> ray_count_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::PrimarySampleSet> sample_set_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::Ray> ray_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::HitInfo> hit_info_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::Ray> feature_ray_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::HitInfo> feature_hit_info_buffer_;
  zivc::SharedBuffer<zivc::uint8b> feature_line_count_buffer_;
  zivc::SharedBuffer<zivc::cl::nanairo::BvhNodeStack> bvh_node_stack_buffer_;
  zivc::SharedBuffer<zivc::cl::uint4> geometry_buffer_;
  zivc::SharedBuffer<zivc::cl::uint4> face_buffer_;
  zivc::SharedBuffer<zivc::cl::uint4> bvh_node_buffer_;
  zivc::SharedBuffer<zivc::uint32b> bvh_map_buffer_;
  zivc::SharedBuffer<zivc::cl::float4> hdr_out_buffer_;
  zivc::SharedBuffer<zivc::cl::uchar4> ldr_out_buffer_;
  zivc::SharedBuffer<zivc::cl::uchar4> ldr_host_buffer_;
  zivc::cl::nanairo::ContextInfo context_info_;
  zivc::cl::nanairo::RenderInfo render_info_;
  zivc::cl::nanairo::CameraInfo camera_info_;
  // Kenrels
  PrimarySampleSpaceKernelT primary_sample_space_kernel_;
  RayGenerationKernelT ray_generation_kernel_;
  FeatureRayGenerationKernelT feature_ray_generation_kernel_;
  RayCastKernelT ray_cast_kernel_;
  FeatureRayCastKernelT feature_ray_cast_kernel_;
  TestKernelT test_kernel_;
  ToneMappingKernelT tone_mapping_kernel_;

  //!
  void destroy()
  {
    // Kernels
    tone_mapping_kernel_.reset();
    test_kernel_.reset();
    feature_ray_cast_kernel_.reset();
    ray_cast_kernel_.reset();
    feature_ray_generation_kernel_.reset();
    ray_generation_kernel_.reset();
    primary_sample_space_kernel_.reset();
    // Buffers
    ldr_host_buffer_.reset();
    ldr_out_buffer_.reset();
    hdr_out_buffer_.reset();
    bvh_map_buffer_.reset();
    bvh_node_buffer_.reset();
    face_buffer_.reset();
    geometry_buffer_.reset();
    bvh_node_stack_buffer_.reset();
    feature_line_count_buffer_.reset();
    feature_hit_info_buffer_.reset();
    feature_ray_buffer_.reset();;
    hit_info_buffer_.reset();
    ray_buffer_.reset();;
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
    context_options.enableVulkanBackend(!options.is_cpu_forced_);
    context_options.enableDebugMode(options.is_debug_mode_ && !options.exclude_vulkan_debug_mode_);

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
    context_info_.setImageResolution({static_cast<zivc::uint16b>(options.image_width_),
                                      static_cast<zivc::uint16b>(options.image_height_)});
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
    // Ray generation kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                ray_generation_kernel,
                                                generateRayKernel,
                                                1);
      ray_generation_kernel_ = device_->createKernel(params);
      ray_generation_kernel_->setName(params.kernelName());
    }
    // Feature ray generation kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                feature_ray_generation_kernel,
                                                generateFeatureRayKernel,
                                                1);
      feature_ray_generation_kernel_ = device_->createKernel(params);
      feature_ray_generation_kernel_->setName(params.kernelName());
    }
    // Ray cast kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                ray_cast_kernel,
                                                castRayKernel,
                                                1);
      ray_cast_kernel_ = device_->createKernel(params);
      ray_cast_kernel_->setName(params.kernelName());
    }
    // Feature ray cast kernel
    {
      const zivc::KernelInitParams params = ZIVC_CREATE_KERNEL_INIT_PARAMS(
                                                feature_ray_cast_kernel,
                                                castFeatureRayKernel,
                                                1);
      feature_ray_cast_kernel_ = device_->createKernel(params);
      feature_ray_cast_kernel_->setName(params.kernelName());
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
    ray_buffer_ = createBuffer<zivc::cl::nanairo::Ray>(n, {zivc::BufferUsage::kPreferDevice}, "RayBuffer");
    hit_info_buffer_ = createBuffer<zivc::cl::nanairo::HitInfo>(n, {zivc::BufferUsage::kPreferDevice}, "HitInfoBuffer");
    feature_ray_buffer_ = createBuffer<zivc::cl::nanairo::Ray>(n, {zivc::BufferUsage::kPreferDevice}, "FeatureRayBuffer");
//    feature_hit_info_buffer_ = createBuffer<zivc::cl::nanairo::HitInfo>(n, {zivc::BufferUsage::kPreferDevice}, "FeatureHitInfoBuffer");
    feature_line_count_buffer_ = createBuffer<zivc::uint8b>(n, {zivc::BufferUsage::kPreferDevice}, "FeatureLineCountBuffer");
    bvh_node_stack_buffer_ = createBuffer<zivc::cl::nanairo::BvhNodeStack>(2 * n, {zivc::BufferUsage::kPreferDevice}, "BvhNodeStackBuffer");
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
    const zivc::cl::float4 value{1.0f, 1.0f, 1.0f, 0.0f};
    zivc::LaunchResult result = zivc::fill(value, data_->hdr_out_buffer_.get(), options);
    result.fence().wait();
  }
  {
    zivc::BufferLaunchOptions options = data_->feature_line_count_buffer_->createOptions();
    options.requestFence(true);
    options.setLabel("ClearLineCount");
    const zivc::uint8b value = 0;
    zivc::LaunchResult result = zivc::fill(value, data_->feature_line_count_buffer_.get(), options);
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

  \param [in] scene No description.
  \param [in] frame No description.
  */
void Renderer::update(const GltfScene& scene, const std::size_t frame)
{
  // Update camera
  {
    const Camera& camera = scene.camera();
    zivc::cl::nanairo::CameraInfo& info = data_->camera_info_;
    // FOV
    info.setFov(camera.yfov_);
    // Transformation
    zivc::cl::float3 translation{};
    std::copy_n(&scene.camera_transl_anim_[6 * frame].x_, 3, &translation.x);
    zivc::cl::nanairo::Quaternion rotation{};
    std::copy_n(&scene.camera_rotate_anim_[6 * frame].x_, 4, &rotation.data_.x);

    const zivc::cl::nanairo::Matrix4x4 m =
        zivc::cl::nanairo::getTranslationMatrix(translation.x, translation.y, translation.z) *
        zivc::cl::nanairo::getRotationMatrix(rotation);
    info.setTransformation(m);
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
  initializeBvh(scene);

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

    // Generate primary ray
    {
      Data::RayGenerationKernelT& kernel = data_->ray_generation_kernel_;
      zivc::KernelLaunchOptions options = kernel->createOptions();
      options.setQueueIndex(0);
      options.setWorkSize({{n}});
      options.requestFence(false);
      options.setLabel("RayGeneration");
      [[maybe_unused]] zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                                               *data_->sample_set_buffer_,
                                                               *data_->ray_buffer_,
                                                               data_->context_info_,
                                                               data_->camera_info_,
                                                               options);
    }

    zivc::ReinterpBuffer face_buffer = data_->face_buffer_->reinterp<zivc::uint32b>();
    zivc::ReinterpBuffer geom_buffer = data_->geometry_buffer_->reinterp<float>();
    zivc::ReinterpBuffer bvh_node_buffer = data_->bvh_node_buffer_->reinterp<zivc::uint32b>();
    // Ray cast kernel
    {
      Data::RayCastKernelT& kernel = data_->ray_cast_kernel_;
      zivc::KernelLaunchOptions options = kernel->createOptions();
      options.setQueueIndex(0);
      options.setWorkSize({{n}});
      options.requestFence(false);
      options.setLabel("RayCast");
      [[maybe_unused]] zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                                               *data_->ray_buffer_,
                                                               face_buffer,
                                                               geom_buffer,
                                                               bvh_node_buffer,
                                                               *data_->bvh_map_buffer_,
                                                               *data_->hit_info_buffer_,
                                                               *data_->bvh_node_stack_buffer_,
                                                               options);
    }

    for (zivc::uint32b feature_ray_count = 0; feature_ray_count < 2; ++feature_ray_count) {
      // Generate feature ray
      {
        Data::FeatureRayGenerationKernelT& kernel = data_->feature_ray_generation_kernel_;
        zivc::KernelLaunchOptions options = kernel->createOptions();
        options.setQueueIndex(0);
        options.setWorkSize({{n}});
        options.requestFence(false);
        options.setLabel("FeatureRayGeneration");
        [[maybe_unused]] zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                                                 *data_->sample_set_buffer_,
                                                                 *data_->ray_buffer_,
                                                                 *data_->hit_info_buffer_,
                                                                 *data_->feature_ray_buffer_,
                                                                 data_->context_info_,
                                                                 data_->camera_info_,
                                                                 feature_ray_count,
                                                                 options);
      }

      // Feature ray cast kernel
      {
        Data::FeatureRayCastKernelT& kernel = data_->feature_ray_cast_kernel_;
        zivc::KernelLaunchOptions options = kernel->createOptions();
        options.setQueueIndex(0);
        options.setWorkSize({{n}});
        options.requestFence(true);
        options.setLabel("FeatureRayCast");
        zivc::LaunchResult result = kernel->run(*data_->ray_count_buffer_,
                                                *data_->hit_info_buffer_,
                                                *data_->feature_ray_buffer_,
                                                face_buffer,
                                                geom_buffer,
                                                bvh_node_buffer,
                                                *data_->bvh_map_buffer_,
                                                *data_->feature_line_count_buffer_,
                                                *data_->bvh_node_stack_buffer_,
                                                options);
        result.fence().wait();
      }
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
                                              //*data_->ray_buffer_,
                                              //*data_->hit_info_buffer_,
                                              //*data_->feature_hit_info_buffer_,
                                              *data_->feature_line_count_buffer_,
                                              *data_->hdr_out_buffer_,
                                              options);
      result.fence().wait();
    }
  }
}

/*!
  \details No detailed description

  \param [in] scene No description.
  */
void Renderer::initializeBvh(const GltfScene& scene) noexcept
{
  using BvhInfoT = zivc::cl::nanairo::BvhInfo;
  using BvhNodeT = zivc::cl::nanairo::BvhNode;
  using Matrix4x4T = zivc::cl::nanairo::Matrix4x4;

  const auto calc_aligned_size = [](const std::size_t s) noexcept
  {
    constexpr std::size_t alignment = sizeof(zivc::cl::uint4);
    return alignment * ((s + alignment - 1) / alignment);
  };
  const auto calc_buffer_size = [](const std::size_t s) noexcept
  {
    return s / sizeof(zivc::uint32b);
  };
  const auto calc_buffer_size4 = [](const std::size_t s) noexcept
  {
    return s / sizeof(zivc::cl::uint4);
  };

  std::size_t geom_size_bytes = 0;
  std::size_t face_size_bytes = 0;
  std::size_t node_size_bytes = 0;

  // TLAS
  BvhInfoT tlas_info{};
  {
    tlas_info.setMaxDepthLevel(scene.bvh_info_.max_level_);
    tlas_info.setNumOfVirtualLeaves(scene.bvh_info_.num_of_virtual_leaves_);
    tlas_info.setTransformation(Matrix4x4T::identity());
    tlas_info.setInvTransformation(Matrix4x4T::identity());
    tlas_info.setGeometryOffset(0);
    // Node size calculation
    std::size_t s = sizeof(BvhInfoT) + sizeof(BvhNodeT) * scene.bvh_node_.size();
    s = calc_aligned_size(s);
    node_size_bytes += s;
  }

  const std::size_t map_size = scene.bvh_leaf_node_.size();
  data_->bvh_map_buffer_ = data_->createBuffer<zivc::uint32b>(map_size, {zivc::BufferUsage::kPreferDevice}, "BvhMapBuffer");
  std::vector<BvhInfoT> blas_info_list{};
  blas_info_list.resize(map_size);

  // BLAS
  {
    zivc::SharedBuffer host_map_buffer = data_->createBuffer<zivc::uint32b>(map_size, {zivc::BufferUsage::kPreferHost, zivc::BufferFlag::kSequentialWritable}, "HostBvhMapBuffer");

    zivc::MappedMemory map_mem = host_map_buffer->mapMemory();
    const std::span meshes = scene.meshList();
    for (std::size_t i = 0; i < scene.bvh_leaf_node_.size(); ++i) {
      const Mesh& mesh = meshes[scene.bvh_leaf_node_[i]];
      BvhInfoT& info = blas_info_list[i];
      //
      {
        info.setMaxDepthLevel(mesh.bvh_info_.max_level_);
        info.setNumOfVirtualLeaves(mesh.bvh_info_.num_of_virtual_leaves_);
        Matrix4x4T m{};
        std::copy_n(mesh.transformation_.cbegin(), 16, &m.m1_.x);
        info.setTransformation(m);
        std::copy_n(mesh.inv_transformation_.cbegin(), 16, &m.m1_.x);
        info.setInvTransformation(m);
        info.setGeometryOffset(calc_buffer_size(geom_size_bytes));
        info.setFaceOffset(calc_buffer_size(face_size_bytes));
      }
      // Geometry size calculation
      {
        std::size_t s = 3 * sizeof(float) * mesh.vertices_.size();
        s = calc_aligned_size(s);
        geom_size_bytes += s;
      }
      // Face size calculation
      {
        std::size_t s = 3 * sizeof(std::uint32_t) * mesh.faces_.size();
        s = calc_aligned_size(s);
        face_size_bytes += s;
      }
      // Node size calculation
      {
        map_mem[i] = calc_buffer_size(node_size_bytes);
        std::size_t s = sizeof(BvhInfoT) + sizeof(BvhNodeT) * mesh.bvh_node_.size();
        s = calc_aligned_size(s);
        node_size_bytes += s;
      }
    }
    map_mem.unmap();

    // Map copy
    {
      zivc::BufferLaunchOptions options = host_map_buffer->createOptions();
      options.requestFence(true);
      options.setLabel("CopyBvhMapToDevice");
      zivc::LaunchResult result = zivc::copy(*host_map_buffer,
                                             data_->bvh_map_buffer_.get(),
                                             options);
      result.fence().wait();
    }
  }

  // Geometry copy
  {
    const std::size_t geom_size = calc_buffer_size4(geom_size_bytes);
    data_->geometry_buffer_ = data_->createBuffer<zivc::cl::uint4>(geom_size, {zivc::BufferUsage::kPreferDevice}, "GeometryBuffer");
    zivc::SharedBuffer host_geom_buffer = data_->createBuffer<zivc::cl::uint4>(geom_size, {zivc::BufferUsage::kPreferHost, zivc::BufferFlag::kSequentialWritable}, "HostGeometryBuffer");

    {
      zivc::ReinterpBuffer host_geom_buffer2 = host_geom_buffer->reinterp<float>();
      zivc::MappedMemory mem = host_geom_buffer2.mapMemory();
      const std::span meshes = scene.meshList();
      for (std::size_t i = 0; i < scene.bvh_leaf_node_.size(); ++i) {
        const Mesh& mesh = meshes[scene.bvh_leaf_node_[i]];
        const BvhInfoT& info = blas_info_list[i];
        const auto* source = reinterpret_cast<const float* const>(mesh.vertices_.data());
        const std::size_t s = 3 * mesh.vertices_.size();
        float* dest = &mem[info.geometryOffset()];
        std::copy_n(source, s, dest);
      }
    }

    {
      zivc::BufferLaunchOptions options = host_geom_buffer->createOptions();
      options.requestFence(true);
      options.setLabel("CopyGeometryToDevice");
      zivc::LaunchResult result = zivc::copy(*host_geom_buffer,
                                             data_->geometry_buffer_.get(),
                                             options);
      result.fence().wait();
    }
  }

  // Face copy
  {
    const std::size_t face_size = calc_buffer_size4(face_size_bytes);
    data_->face_buffer_ = data_->createBuffer<zivc::cl::uint4>(face_size, {zivc::BufferUsage::kPreferDevice}, "FaceBuffer");
    zivc::SharedBuffer host_face_buffer = data_->createBuffer<zivc::cl::uint4>(face_size, {zivc::BufferUsage::kPreferHost, zivc::BufferFlag::kSequentialWritable}, "HostFaceBuffer");

    {
      zivc::ReinterpBuffer host_face_buffer2 = host_face_buffer->reinterp<zivc::uint32b>();
      zivc::MappedMemory mem = host_face_buffer2.mapMemory();
      const std::span meshes = scene.meshList();
      for (std::size_t i = 0; i < scene.bvh_leaf_node_.size(); ++i) {
        const Mesh& mesh = meshes[scene.bvh_leaf_node_[i]];
        const BvhInfoT& info = blas_info_list[i];
        zivc::uint32b* dest = &mem[info.faceOffset()];
        for (std::size_t j = 0; j < mesh.bvh_leaf_node_.size(); ++j) {
          const U3& face = mesh.faces_[mesh.bvh_leaf_node_[j]];
          std::copy_n(&face.x_, 3, dest + 3 * j);
        }
      }
    }

    {
      zivc::BufferLaunchOptions options = host_face_buffer->createOptions();
      options.requestFence(true);
      options.setLabel("CopyFaceToDevice");
      zivc::LaunchResult result = zivc::copy(*host_face_buffer,
                                             data_->face_buffer_.get(),
                                             options);
      result.fence().wait();
    }
  }

  {
    const std::size_t node_size = calc_buffer_size4(node_size_bytes);
    data_->bvh_node_buffer_ = data_->createBuffer<zivc::cl::uint4>(node_size, {zivc::BufferUsage::kPreferDevice}, "BvhNodeBuffer");
    zivc::SharedBuffer host_node_buffer = data_->createBuffer<zivc::cl::uint4>(node_size, {zivc::BufferUsage::kPreferHost, zivc::BufferFlag::kSequentialWritable}, "HostBvhNodeBuffer");

    {
      zivc::ReinterpBuffer host_node_buffer2 = host_node_buffer->reinterp<float>();
      zivc::MappedMemory mem = host_node_buffer2.mapMemory();

      std::size_t offset = 0;
      // TLAS info
      {
        const BvhInfoT& info = tlas_info;
        const auto* source = reinterpret_cast<const float* const>(&info);
        const std::size_t s = sizeof(info) / sizeof(float);
        std::copy_n(source, s, mem.begin());
        offset += s;
      }
      {
        const auto* source = reinterpret_cast<const float* const>(scene.bvh_node_.data());
        const std::size_t s = (sizeof(BvhNodeT) * scene.bvh_node_.size()) / sizeof(float);
        std::copy_n(source, s, &mem[offset]);
        offset += calc_buffer_size(calc_aligned_size(sizeof(float) * s));
      }

      // BLAS
      const std::span meshes = scene.meshList();
      for (std::size_t i = 0; i < scene.bvh_leaf_node_.size(); ++i) {
        const Mesh& mesh = meshes[scene.bvh_leaf_node_[i]];
        // info
        {
          const BvhInfoT& info = blas_info_list[i];
          const auto* source = reinterpret_cast<const float* const>(&info);
          const std::size_t s = sizeof(info) / sizeof(float);
          std::copy_n(source, s, &mem[offset]);
          offset += s;
        }
        {
          const auto* source = reinterpret_cast<const float* const>(mesh.bvh_node_.data());
          const std::size_t s = (sizeof(BvhNodeT) * mesh.bvh_node_.size()) / sizeof(float);
          std::copy_n(source, s, &mem[offset]);
          offset += calc_buffer_size(calc_aligned_size(sizeof(float) * s));
        }
      }
    }

    {
      zivc::BufferLaunchOptions options = host_node_buffer->createOptions();
      options.requestFence(true);
      options.setLabel("CopyBvhNodeToDevice");
      zivc::LaunchResult result = zivc::copy(*host_node_buffer,
                                             data_->bvh_node_buffer_.get(),
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
