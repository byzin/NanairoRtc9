# file: CMakeLists.txt
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
# 

cmake_minimum_required(VERSION 3.26)


function(Nanairo_initNanairoOptions)
  #  set(option_description "Enable Vulkan backend.")
  #  Zivc_setBooleanOption(ZIVC_ENABLE_VULKAN_BACKEND ON ${option_description})
  #
  #  set(option_description "Load Vulkan library dynamically at runtime.")
  #  Zivc_setBooleanOption(ZIVC_DYNAMIC_VULKAN_LOADING ON ${option_description})
  #
  #  set(option_description "Enable Zivc to build VMA implementation.")
  #  Zivc_setBooleanOption(ZIVC_ENABLE_VMA_IMPLEMENTATION ON ${option_description})
  #
  #  set(option_description "Zivc uses own 'clspv' instead of existing 'clspv'.")
  #  Zivc_setBooleanOption(ZIVC_USES_OWN_CLSPV OFF ${option_description})
  #
  #  set(option_description "Specify the default SPIR-V version used in Zivc.")
  #  Zivc_setStringOption(ZIVC_DEFAULT_SPIRV_VERSION "1.5" ${option_description})
  #
  #  set(option_description "Bake Vulkan SPIR-V kernels into it's library.")
  #  Zivc_setBooleanOption(ZIVC_BAKE_KERNELS OFF ${option_description})
  #
  #  set(option_description "Enable SPIR-V analysis.")
  #  Zivc_setBooleanOption(ZIVC_ENABLE_SPIRV_ANALYSIS ON ${option_description})
  #
  #  include("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/cmake/kernel.cmake")
  #  Zivc_initZivcKernelOptions()
endfunction(Nanairo_initNanairoOptions)


function(Nanairo_getNanairoFlags nanairo_compile_flags nanairo_linker_flags nanairo_definitions)
  #  include("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/cmake/kernel.cmake")
  #  Zivc_getZivcKernelFlags(compile_flags definitions)
  #
  #  if(ZIVC_ENABLE_VULKAN_BACKEND)
  #    list(APPEND definitions ZIVC_ENABLE_VULKAN_BACKEND=1)
  #    # Dynamic vulkan loading
  #    if(ZIVC_DYNAMIC_VULKAN_LOADING)
  #      list(APPEND definitions ZIVC_DYNAMIC_VULKAN_LOADING=1)
  #    endif()
  #    # Vulkan Memory Allocator
  #    if(ZIVC_ENABLE_VMA_IMPLEMENTATION)
  #      list(APPEND definitions ZIVC_ENABLE_VMA_IMPLEMENTATION=1)
  #    endif()
  #    if(ZIVC_BAKE_KERNELS)
  #      list(APPEND definitions ZIVC_BAKE_KERNELS=1)
  #    endif()
  #  endif()
  #
  #  # C++
  #  list(APPEND definitions ZIVC_CL_CPU=1)
  #  # Suppress warnings
  #  if(Z_VISUAL_STUDIO)
  #    list(APPEND definitions _CRT_SECURE_NO_WARNINGS)
  #  endif()

  # Output variables
  set(${nanairo_compile_flags} ${compile_flags} PARENT_SCOPE)
  set(${nanairo_definitions} ${definitions} PARENT_SCOPE)
endfunction(Nanairo_getNanairoFlags)


function(Nanairo_getNanairoWarningFlags nanairo_warning_flags)
  set(warning_flags "")

  # Suppress warnings
  if(Z_CLANG)
    list(APPEND warning_flags
                             )

    if(Z_VISUAL_STUDIO)
      list(APPEND warning_flags
                               )
    endif()
  elseif(Z_GCC)
    list(APPEND warning_flags
                             )
  elseif(Z_VISUAL_STUDIO)
    list(APPEND warning_flags 
                             /wd6326 # Potential comparison of a constant with another constant
                             )
  endif()

  set(${nanairo_warning_flags} ${warning_flags} PARENT_SCOPE)
endfunction(Nanairo_getNanairoWarningFlags)


#function(Zivc_addZivcKernels)
#  cmake_path(SET zivc_path ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
#  include("${zivc_path}/cmake/kernel.cmake")
#
#  cmake_path(SET kernel_set_dir "${zivc_path}/internal_kernel/zivc")
#  file(GLOB_RECURSE kernel_sources ${kernel_set_dir}/kernel/*.cl)
#  set(kernel_set_name "zivc_internal_kernel")
#  Zivc_addKernelSet(${kernel_set_name} ${PROJECT_VERSION} target_base_name
#      SOURCE_FILES ${kernel_sources})
#  add_library(Zivc::InternalKernel ALIAS ${target_base_name})
#endfunction(Zivc_addZivcKernels)


#
macro(Nanairo_setNanairoProject)
  set(project_description "The path tracing renderer for RTC9.")
  project(Nanairo VERSION ${NanairoProject_VERSION} DESCRIPTION "${project_description}" LANGUAGES CXX)
  message(STATUS "${PROJECT_NAME} version: ${PROJECT_VERSION}")

  cmake_path(SET zivc_path ${PROJECT_SOURCE_DIR}/../dependencies/Zivc/source/zivc)
  include("${zivc_path}/cmake/general.cmake")
  include("${zivc_path}/cmake/platform.cmake")
  Zivc_getPlatformFlags(platform_definitions)
  Zivc_setVariablesOnCMake(${platform_definitions})
  Nanairo_initNanairoOptions()

  # Check dependencies
  Zivc_checkTarget(CpuFeature::cpu_features)
  Zivc_checkTarget(CLI11::CLI11)

  cmake_path(SET nanairo_cmd_dir "${PROJECT_SOURCE_DIR}/cmd")
  #cmake_path(SET zivc_include_dir "${zivc_core_dir}/include")
  # Make zivc configuration header file
  #cmake_path(SET zivc_config_dir "${PROJECT_BINARY_DIR}/include/zivc")
  #file(MAKE_DIRECTORY "${zivc_config_dir}")
  #configure_file("${zivc_core_dir}/zivc/zivc_config-inl.hpp.in"
  #               "${zivc_config_dir}/zivc_config-inl.hpp"
  #               @ONLY)

  # Create Zivc library
  file(GLOB_RECURSE nanairo_source_files ${nanairo_cmd_dir}/*[.hpp|.cpp])
  #list(APPEND zivc_source_files "${zivc_config_dir}/zivc_config-inl.hpp")
  source_group(${PROJECT_NAME} FILES ${nanairo_source_files})

  # Compiler flags
  include("${zivc_path}/cmake/compiler.cmake")
  Zivc_getCxxWarningFlags(cxx_compile_warning_flags)
  Zivc_getSanitizerFlags(compile_sanitizer_flags linker_sanitizer_flags)
  Nanairo_getNanairoFlags(nanairo_compile_flags nanairo_linker_flags nanairo_definitions)
  Nanairo_getNanairoWarningFlags(nanairo_warning_flags)
  # Get architecture
  Zivc_checkTarget(Zivc::Zivc)
  # Nanairo
  add_executable(${PROJECT_NAME} "${nanairo_source_files}")
  set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20
                                                   CXX_STANDARD_REQUIRED ON)
  target_compile_options(${PROJECT_NAME} PUBLIC ${nanairo_compile_flags}
                                         PRIVATE ${cxx_compile_warning_flags}
                                                 ${compile_sanitizer_flags}
                                                 ${nanairo_warning_flags})
  target_link_libraries(${PROJECT_NAME} PUBLIC Zivc::Zivc
                                        PRIVATE CpuFeature::cpu_features)
  target_link_options(${PROJECT_NAME} PUBLIC ${nanairo_linker_flags}
                                      PRIVATE ${linker_sanitizer_flags})
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${nanairo_definitions})
  target_include_directories(${PROJECT_NAME} PUBLIC "${nanairo_include_dir}"
                                                    "${PROJECT_BINARY_DIR}/include")
  Zivc_enableIpo(${PROJECT_NAME})

  #
  set(clang_tidy_exclusion_checks bugprone-exception-escape
                                  misc-misplaced-const
                                  misc-no-recursion
                                  readability-identifier-length
                                  readability-static-accessed-through-instance
                                  readability-uppercase-literal-suffix)
  Zivc_setStaticAnalyzer(${PROJECT_NAME}
                         CLANG_TIDY_EXCLUSION_CHECKS ${clang_tidy_exclusion_checks})
  Zivc_createLinkToTarget(${PROJECT_NAME} ${PROJECT_BINARY_DIR})
endmacro(Nanairo_setNanairoProject)


##
Nanairo_setNanairoProject()
