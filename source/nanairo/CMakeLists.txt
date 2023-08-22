# file: CMakeLists.txt
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
# 

cmake_minimum_required(VERSION 3.26)


function(Nanairo_initNanairoOptions)
endfunction(Nanairo_initNanairoOptions)


function(Nanairo_getNanairoFlags nanairo_compile_flags nanairo_linker_flags nanairo_definitions)
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
  Zivc_checkTarget(Tinygltf::tinygltf)
  Zivc_checkTarget(Spng::spng)

  cmake_path(SET nanairo_cmd_dir "${PROJECT_SOURCE_DIR}/cmd")

  # Create Zivc library
  file(GLOB_RECURSE nanairo_source_files ${nanairo_cmd_dir}/*[.hpp|.cpp])
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
                                        PRIVATE CpuFeature::cpu_features
                                                CLI11::CLI11
                                                Tinygltf::tinygltf
                                                Spng::spng)
  target_link_options(${PROJECT_NAME} PUBLIC ${nanairo_linker_flags}
                                      PRIVATE ${linker_sanitizer_flags})
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${nanairo_definitions})
  target_include_directories(${PROJECT_NAME} PUBLIC "${nanairo_include_dir}"
                                                    "${PROJECT_BINARY_DIR}/include")
  Zivc_enableIpo(${PROJECT_NAME})

  # Add kernel sets
  include("${zivc_path}/cmake/kernel.cmake")
  cmake_path(SET kernel_dir NORMALIZE "${PROJECT_SOURCE_DIR}/kernel")
  cmake_path(SET common_header_dir NORMALIZE "${kernel_dir}/include")
  ## Primary sample space kernel
  cmake_path(SET kernel_set_dir NORMALIZE "${kernel_dir}/primary_sample_space_kernel")
  file(GLOB_RECURSE kernel_sources ${kernel_set_dir}/*.cl)
  Zivc_addKernelSet(primary_sample_space_kernel ${PROJECT_VERSION} kernel_name
      SOURCE_FILES ${kernel_sources}
      INCLUDE_DIRS ${common_header_dir})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${kernel_name})
  ## Ray cast kernel
  cmake_path(SET kernel_set_dir NORMALIZE "${kernel_dir}/ray_cast_kernel")
  file(GLOB_RECURSE kernel_sources ${kernel_set_dir}/*.cl)
  Zivc_addKernelSet(ray_cast_kernel ${PROJECT_VERSION} kernel_name
      SOURCE_FILES ${kernel_sources}
      INCLUDE_DIRS ${common_header_dir})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${kernel_name})
  ## Tone mapping kernel
  cmake_path(SET kernel_set_dir NORMALIZE "${kernel_dir}/tone_mapping_kernel")
  file(GLOB_RECURSE kernel_sources ${kernel_set_dir}/*.cl)
  Zivc_addKernelSet(tone_mapping_kernel ${PROJECT_VERSION} kernel_name
      SOURCE_FILES ${kernel_sources}
      INCLUDE_DIRS ${common_header_dir})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${kernel_name})
  ## Test
  cmake_path(SET kernel_set_dir NORMALIZE "${kernel_dir}/test_kernel")
  file(GLOB_RECURSE kernel_sources ${kernel_set_dir}/*.cl)
  Zivc_addKernelSet(test_kernel ${PROJECT_VERSION} kernel_name
      SOURCE_FILES ${kernel_sources}
      INCLUDE_DIRS ${common_header_dir})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${kernel_name})


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

  # Installation settings
  include("${zivc_path}/cmake/packaging.cmake")
  ## Save the dependency list
  Zivc_getSaveDependencyCode(${PROJECT_NAME} "${PROJECT_BINARY_DIR}" "" "" "" dependency_code)
  install(CODE ${dependency_code}
          COMPONENT renderer)
  ## Install actual binary
  install(TARGETS ${PROJECT_NAME}
          DESTINATION "${CMAKE_INSTALL_BINDIR}"
          COMPONENT renderer)
  ## Dependencies
  Zivc_getDeployingNonSystemDependencyCode(${PROJECT_NAME} "${CMAKE_INSTALL_LIBDIR}" "" "" "" deploy_code)
  install(CODE ${deploy_code}
          COMPONENT renderer)
  Zivc_getDeployingVulkanDependencyCode("${CMAKE_INSTALL_LIBDIR}" share vulkan_dep_code vulkan_envs)
  install(CODE ${vulkan_dep_code}
          COMPONENT renderer)
  ## Set vulkan installed directory
  cmake_path(SET vulkan_sdk_dir "\\\${Z_SCRIPT_DIR}/..")
  cmake_path(NATIVE_PATH vulkan_sdk_dir vulkan_sdk_dir)
  list(INSERT vulkan_envs 0 "VULKAN_SDK=${vulkan_sdk_dir}")
  ## Runner script
  Zivc_getMakeSimpleRunScriptCode(${PROJECT_NAME} "../${CMAKE_INSTALL_BINDIR}" "../${CMAKE_INSTALL_LIBDIR}" "${vulkan_envs}" "runUnitTest" "${PROJECT_NAME}" run_script_code)
  install(CODE ${run_script_code}
          COMPONENT renderer)
endmacro(Nanairo_setNanairoProject)


##
Nanairo_setNanairoProject()
