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
  addZstd("${PROJECT_BINARY_DIR}/Zstd")
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

  cmake_path(SET source_scene_path1 NORMALIZE "${source_scene_dir}/scene.glb.zip")
  cmake_path(SET dest_scene_path1 NORMALIZE "${dest_scene_dir}/scene.glb")
  cmake_path(SET source_scene_path2 NORMALIZE "${source_scene_dir}/TestHouse.glb.zip")
  cmake_path(SET dest_scene_path2 NORMALIZE "${dest_scene_dir}/TestHouse.glb")
  cmake_path(SET source_scene_path3 NORMALIZE "${source_scene_dir}/TestPlane.glb.zip")
  cmake_path(SET dest_scene_path3 NORMALIZE "${dest_scene_dir}/TestPlane.glb")
  cmake_path(SET source_scene_path4 NORMALIZE "${source_scene_dir}/TestPlane2.glb.zip")
  cmake_path(SET dest_scene_path4 NORMALIZE "${dest_scene_dir}/TestPlane2.glb")
  add_custom_command(OUTPUT "${dest_scene_path1}"
    COMMAND ${CMAKE_COMMAND} -E tar xfz ${source_scene_path1}
    WORKING_DIRECTORY ${dest_scene_dir}
    DEPENDS ${source_scene_path1}
    COMMENT "Unzip scene file '${source_scene_path1}'")
  add_custom_command(OUTPUT "${dest_scene_path2}"
    COMMAND ${CMAKE_COMMAND} -E tar xfz ${source_scene_path2}
    WORKING_DIRECTORY ${dest_scene_dir}
    DEPENDS ${source_scene_path2}
    COMMENT "Unzip scene file '${source_scene_path2}'")
  add_custom_command(OUTPUT "${dest_scene_path3}"
    COMMAND ${CMAKE_COMMAND} -E tar xfz ${source_scene_path3}
    WORKING_DIRECTORY ${dest_scene_dir}
    DEPENDS ${source_scene_path3}
    COMMENT "Unzip scene file '${source_scene_path3}'")
  add_custom_command(OUTPUT "${dest_scene_path4}"
    COMMAND ${CMAKE_COMMAND} -E tar xfz ${source_scene_path4}
    WORKING_DIRECTORY ${dest_scene_dir}
    DEPENDS ${source_scene_path4}
    COMMENT "Unzip scene file '${source_scene_path4}'")
  add_custom_target(Resources ALL
    DEPENDS ${dest_scene_path1} ${dest_scene_path2} ${dest_scene_path3} ${dest_scene_path4}
    COMMENT "Preparing resource files")

  # Test
  cmake_path(SET test_dir NORMALIZE "${PROJECT_BINARY_DIR}/test")
  file(MAKE_DIRECTORY ${test_dir})
  enable_testing()
  add_test(NAME Nanairo-single-vulkan
               COMMAND Nanairo ${dest_scene_path} --min-frame 0 --max-frame 1 --time-budget 4000 --debug
               WORKING_DIRECTORY ${test_dir})
  add_test(NAME Nanairo-single-cpu
               COMMAND Nanairo ${dest_scene_path} --min-frame 0 --max-frame 1 --time-budget 4000 --debug --force-cpu
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

# Install
if(PROJECT_IS_TOP_LEVEL)
  install(PROGRAMS ${PROJECT_SOURCE_DIR}/install/fps.txt
                   ${PROJECT_SOURCE_DIR}/install/run.ps1
                   ${PROJECT_SOURCE_DIR}/install/run.py
          DESTINATION .)
  install(DIRECTORY ${PROJECT_BINARY_DIR}/resource
          DESTINATION .)
endif()
