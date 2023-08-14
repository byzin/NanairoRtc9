# file: CMakeLists.txt
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
#

cmake_minimum_required(VERSION 3.26)


project(NanairoProject VERSION 0.0.1 LANGUAGES CXX)

cmake_path(SET zivc_path "${PROJECT_SOURCE_DIR}/source/dependencies/Zivc")
if(PROJECT_IS_TOP_LEVEL)
  ## Cmake configurations
  set(CMAKE_CONFIGURATION_TYPES "Debug" "RelWithDebInfo" "Release")
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
  endif()
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  include(InstallRequiredSystemLibraries)
  # Print system environment information
  include("${zivc_path}/source/zivc/cmake/general.cmake")
  message(STATUS "System information:")
  Zivc_printSystemInfo("  + ")
  ## Project options
  include("${PROJECT_SOURCE_DIR}/cmake/option.cmake")
  initProjectOptions()
  Zivc_restrictBuildDirectory("${PROJECT_SOURCE_DIR}/build")

  # Add dependencies
  include("${PROJECT_SOURCE_DIR}/cmake/project.cmake")
  Zivc_setInternalValue(BUILD_TESTING OFF)
  Zivc_setInternalValue(BUILD_SHARED_LIBS OFF)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  addZisc("${PROJECT_BINARY_DIR}/Zisc")
  setZiscAlias()
  addCpuFeatures("${PROJECT_BINARY_DIR}/CpuFeatures")
  addCli11("${PROJECT_BINARY_DIR}/CLI11")
  addTinygltf("${PROJECT_BINARY_DIR}/Tinygltf")
  addZlib("${PROJECT_BINARY_DIR}/Zlib")
  addSpng("${PROJECT_BINARY_DIR}/Spng")
  findVulkan()
  addZivc("${PROJECT_BINARY_DIR}/Zivc")
  setZivcAlias()
endif()

# Build Nanairo
cmake_path(SET nanairo_path "${PROJECT_SOURCE_DIR}/source/nanairo")
add_subdirectory("${nanairo_path}" "${PROJECT_BINARY_DIR}/Nanairo")

if(PROJECT_IS_TOP_LEVEL)
  # Packaging
  include("${PROJECT_SOURCE_DIR}/cmake/packaging.cmake")
  packageProject()
endif()

# Resources and tests
if(PROJECT_IS_TOP_LEVEL)
  cmake_path(SET source_scene_dir NORMALIZE "${PROJECT_SOURCE_DIR}/resource/scene")
  cmake_path(SET dest_scene_dir NORMALIZE "${PROJECT_BINARY_DIR}/resource/scene")
  file(MAKE_DIRECTORY ${dest_scene_dir})

  cmake_path(SET source_scene_path NORMALIZE "${source_scene_dir}/scene.glb.zip")
  cmake_path(SET dest_scene_path NORMALIZE "${dest_scene_dir}/scene.glb")
  add_custom_command(OUTPUT "${dest_scene_path}"
    COMMAND ${CMAKE_COMMAND} -E tar xfz ${source_scene_path}
    WORKING_DIRECTORY ${dest_scene_dir}
    DEPENDS ${source_scene_path}
    COMMENT "Unzip scene file '${source_scene_path}'")
  add_custom_target(Resources ALL
    DEPENDS ${dest_scene_path}
    COMMENT "Preparing resource files")

  # Test
  cmake_path(SET test_dir NORMALIZE "${PROJECT_BINARY_DIR}/test")
  file(MAKE_DIRECTORY ${test_dir})
  enable_testing()
  add_test(NAME Nanairo-single-vulkan
               COMMAND Nanairo ${dest_scene_path} --min-frame=0 --max-frame=1 --debug
               WORKING_DIRECTORY ${test_dir})
  add_test(NAME Nanairo-single-cpu
               COMMAND Nanairo ${dest_scene_path} --min-frame=0 --max-frame=1 --debug --force-cpu
               WORKING_DIRECTORY ${test_dir})
  add_test(NAME Nanairo-animation-vulkan
               COMMAND Nanairo ${dest_scene_path} --debug
               WORKING_DIRECTORY ${test_dir})
  add_test(NAME Nanairo-animation-cpu
               COMMAND Nanairo ${dest_scene_path} --debug --force-cpu
               WORKING_DIRECTORY ${test_dir})
  ## Test properties
  set(test_env_list "")
  list(APPEND test_env_list ZIVC_SUPPRESS_TRIVIAL_WARNINGS=1)
  set_tests_properties(Nanairo-single-vulkan
                       Nanairo-single-cpu
                       Nanairo-animation-vulkan
                       Nanairo-animation-cpu
                       PROPERTIES ENVIRONMENT "${test_env_list}")
endif()
