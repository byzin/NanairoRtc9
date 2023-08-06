# file: project.cmake
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
# 


# Build Zisc
function(addZisc binary_dir)
  if(TARGET Zisc::Zisc)
    return()
  else()
    message(STATUS "Add Zisc subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zisc_path "${dependencies_dir}/Zisc")
  cmake_path(SET zivc_path "${dependencies_dir}/Zivc/source/zivc")
  # Add Zisc
  include("${zivc_path}/cmake/general.cmake")
  Zivc_checkSubmodule(${zisc_path})
  add_subdirectory(${zisc_path} ${binary_dir})
endfunction(addZisc)


# Build Zivc
function(addZivc binary_dir)
  if(TARGET Zivc::Zivc)
    return()
  else()
    message(STATUS "Add Zivc subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zivc_path "${dependencies_dir}/Zivc")
  # Add Zivc
  include("${zivc_path}/source/zivc/cmake/general.cmake")
  Zivc_checkSubmodule(${zivc_path})
  Zivc_setInternalValue(ZIVC_BUILD_EXAMPLES OFF)
  Zivc_setInternalValue(ZIVC_BUILD_TESTS OFF)
  add_subdirectory(${zivc_path} ${binary_dir})
endfunction(addZivc)


#
function(addCpuFeatures binary_dir)
  if(TARGET CpuFeature::cpu_features)
    return()
  else()
    message(STATUS "Add CpuFeatures subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zivc_path "${dependencies_dir}/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  include("${zivc_path}/cmake/platform.cmake")
  Zivc_getPlatformFlags(platform_definitions)
  Zivc_setVariablesOnCMake(${platform_definitions})
  # Add CpuFeatures
  cmake_path(SET cpu_features_path NORMALIZE "${dependencies_dir}/cpu_features")
  Zivc_checkSubmodule("${cpu_features_path}")
  add_subdirectory("${cpu_features_path}" "${binary_dir}" EXCLUDE_FROM_ALL)
  Zivc_checkTarget(CpuFeature::cpu_features)
  # Set properties
  set_target_properties(cpu_features PROPERTIES C_STANDARD 17
                                                C_STANDARD_REQUIRED ON)
  # Supress warnings
  set(cpu_warning_flags "")
  if(Z_VISUAL_STUDIO)
    list(APPEND cpu_warning_flags /w)
  elseif(Z_CLANG AND NOT Z_APPLE_CLANG)
    list(APPEND cpu_warning_flags -Wno-unused-command-line-argument
                                  -Wno-unused-but-set-variable
                                  )
  endif()
  target_compile_options(cpu_features PRIVATE ${cpu_warning_flags})
endfunction(addCpuFeatures)


# Build CLI11
function(addCli11 binary_dir)
  if(TARGET CLI11::CLI11)
    return()
  else()
    message(STATUS "Add CLI11 subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET zivc_path "${project_dir}/source/dependencies/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  # Add CLI11
  Zivc_setInternalValue(CLI11_WARNINGS_AS_ERRORS OFF)
  Zivc_setInternalValue(CLI11_SINGLE_FILE OFF)
  Zivc_setInternalValue(CLI11_SANITIZERS OFF)
  Zivc_setInternalValue(CLI11_BUILD_DOCS OFF)
  Zivc_setInternalValue(CLI11_BUILD_TESTS OFF)
  Zivc_setInternalValue(CLI11_BUILD_EXAMPLES OFF)
  Zivc_setInternalValue(CLI11_BUILD_EXAMPLES_JSON OFF)
  Zivc_setInternalValue(CLI11_SINGLE_FILE_TESTS OFF)
  Zivc_setInternalValue(CLI11_INSTALL OFF)
  Zivc_setInternalValue(CLI11_FORCE_LIBCXX OFF)
  Zivc_setInternalValue(CLI11_CUDA_TESTS OFF)
  Zivc_setInternalValue(CLI11_CLANG_TIDY_OPTIONS "")
  Zivc_setInternalValue(CLI11_PRECOMPILED OFF)
  Zivc_setInternalValue(CLI11_SANITIZERS OFF)
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET cli11_source_path "${dependencies_dir}/CLI11")
  Zivc_checkSubmodule("${cli11_source_path}")
  add_subdirectory("${cli11_source_path}" "${binary_dir}" EXCLUDE_FROM_ALL)
  Zivc_checkTarget(CLI11::CLI11)
endfunction(addCli11)


#
function(addTinygltf binary_dir)
  if(TARGET Tinygltf::tinygltf)
    return()
  else()
    message(STATUS "Add tinygltf subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET zivc_path "${project_dir}/source/dependencies/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  # Add tinygltf
  Zivc_setInternalValue(TINYGLTF_BUILD_LOADER_EXAMPLE OFF)
  Zivc_setInternalValue(TINYGLTF_BUILD_GL_EXAMPLES OFF)
  Zivc_setInternalValue(TINYGLTF_BUILD_VALIDATOR_EXAMPLE OFF)
  Zivc_setInternalValue(TINYGLTF_BUILD_BUILDER_EXAMPLE OFF)
  Zivc_setInternalValue(TINYGLTF_HEADER_ONLY ON)
  Zivc_setInternalValue(TINYGLTF_INSTALL OFF)
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET tinygltf_path NORMALIZE "${dependencies_dir}/tinygltf")
  Zivc_checkSubmodule("${tinygltf_path}")
  add_subdirectory("${tinygltf_path}" "${binary_dir}" EXCLUDE_FROM_ALL)
  Zivc_checkTarget(tinygltf)
  add_library(Tinygltf::tinygltf ALIAS tinygltf)
  # Set properties 
  set_target_properties(tinygltf PROPERTIES CXX_STANDARD 20
                                            CXX_STANDARD_REQUIRED ON)
  target_compile_definitions(tinygltf INTERFACE TINYGLTF_NO_STB_IMAGE_WRITE=1
                                                TINYGLTF_NO_INCLUDE_STB_IMAGE_WRITE=1
                                                TINYGLTF_USE_CPP14=1
                                                STBI_NO_SIMD=1
                                                STBI_NO_PSD=1
                                                STBI_NO_GIF=1)
endfunction(addTinygltf)


#
function(addZlib binary_dir)
  if(TARGET Zlib::zlib)
    return()
  else()
    message(STATUS "Add zlib subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET zivc_path "${project_dir}/source/dependencies/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  include("${zivc_path}/cmake/platform.cmake")
  Zivc_getPlatformFlags(platform_definitions)
  Zivc_setVariablesOnCMake(${platform_definitions})
  # Add zlib
  Zivc_setInternalValue(ASM686 OFF)
  Zivc_setInternalValue(AMD64 OFF)
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zlib_path NORMALIZE "${dependencies_dir}/zlib")
  Zivc_checkSubmodule("${zlib_path}")
  add_subdirectory("${zlib_path}" "${binary_dir}" EXCLUDE_FROM_ALL)
  Zivc_checkTarget(zlibstatic)
  add_library(Zlib::zlib ALIAS zlibstatic)
  # Set properties
  set_target_properties(zlibstatic PROPERTIES C_STANDARD 17
                                              C_STANDARD_REQUIRED ON)
  target_include_directories(zlibstatic INTERFACE ${zlib_path} ${binary_dir})
  # Supress warnings
  set(zlib_warning_flags "")
  if(Z_VISUAL_STUDIO)
    list(APPEND zlib_warning_flags /w)
  elseif(Z_CLANG AND NOT Z_APPLE_CLANG)
    list(APPEND zlib_warning_flags -Wno-deprecated-non-prototype
                                   )
  endif()
  target_compile_options(zlibstatic PRIVATE ${zlib_warning_flags})
endfunction(addZlib)


# Build spng
function(addSpng binary_dir)
  if(TARGET Spng::spng)
    return()
  else()
    message(STATUS "Add spng subdirectory.")
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET zivc_path "${project_dir}/source/dependencies/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  # Add spng
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET spng_source_path "${dependencies_dir}/libspng_cmake")
  Zivc_checkSubmodule("${spng_source_path}")
  add_subdirectory("${spng_source_path}" "${binary_dir}" EXCLUDE_FROM_ALL)
  Zivc_checkTarget(spng)
  add_library(Spng::spng ALIAS spng)
  # Set properties
  set_target_properties(spng PROPERTIES C_STANDARD 17
                                        C_STANDARD_REQUIRED ON)
endfunction(addSpng)


#
function(findVulkan)
  Zivc_setInternalValue(ZIVC_ENABLE_VULKAN_BACKEND ${Z_ENABLE_VULKAN_BACKEND})
  if(NOT Z_ENABLE_VULKAN_BACKEND)
    return()
  endif()

  if(TARGET Zivc::Vulkan)
    return()
  else()
    message(STATUS "Find Vulkan library.")
  endif()

  # Load vulkan module
  find_package(Vulkan 1.3.250 REQUIRED)

  # Set vulkan target
  if(ZIVC_DYNAMIC_VULKAN_LOADING)
    add_library(Zivc::Vulkan ALIAS Vulkan::Headers)
  else()
    add_library(Zivc::Vulkan ALIAS Vulkan::Vulkan)
  endif()
endfunction(findVulkan)


# Zisc
function(setZiscAlias)
  if(TARGET Zisc::Zisc)
    return()
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zivc_path "${dependencies_dir}/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  include("${zivc_path}/cmake/platform.cmake")
  Zivc_getPlatformFlags(platform_definitions)
  Zivc_setVariablesOnCMake(${platform_definitions})

  include("${zivc_path}/cmake/compiler.cmake")
  Zivc_getArchitectureName(TRUE architecture)
  Zivc_getArchitectureTargetName("Zisc" ${architecture} zisc_name)
  Zivc_checkTarget(${zisc_name})
  add_library(Zisc::Zisc ALIAS ${zisc_name})
endfunction(setZiscAlias)


# Zisc
function(setZivcAlias)
  if(TARGET Zivc::Zivc)
    return()
  endif()

  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
  cmake_path(SET dependencies_dir NORMALIZE "${project_dir}/source/dependencies")
  cmake_path(SET zivc_path "${dependencies_dir}/Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  include("${zivc_path}/cmake/platform.cmake")
  Zivc_getPlatformFlags(platform_definitions)
  Zivc_setVariablesOnCMake(${platform_definitions})

  include("${zivc_path}/cmake/compiler.cmake")
  Zivc_getArchitectureName(TRUE architecture)
  Zivc_getArchitectureTargetName("Zivc" ${architecture} zisc_name)
  Zivc_checkTarget(${zisc_name})
  add_library(Zivc::Zivc ALIAS ${zisc_name})
endfunction(setZivcAlias)


# GoogleTest
#function(addGoogleTest binary_dir)
#  if(TARGET GTest::gtest)
#    return()
#  else()
#    message(STATUS "Add GoogleTest subdirectory.")
#  endif()
#
#
#  # Add googletest
#  cmake_path(SET project_dir NORMALIZE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/..")
#  cmake_path(SET zivc_path "${project_dir}/source/zivc")
#  cmake_path(SET dependencies_dir "${project_dir}/test/dependencies")
#  cmake_path(SET googletest_path "${dependencies_dir}/googletest")
#  include("${zivc_path}/cmake/general.cmake")
#  Zivc_checkSubmodule(${googletest_path})
#  Zivc_addGoogleTest(${googletest_path} ${binary_dir})
#  Zivc_checkTarget(gtest)
#  #
#  include("${zivc_path}/cmake/compiler.cmake")
#  Zivc_populateTargetOptions(Zisc::Zisc gtest)
#  Zivc_checkTarget(GTest::gtest)
#endfunction(addGoogleTest)
