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
  findVulkan()
  addZivc("${PROJECT_BINARY_DIR}/Zivc")
  setZivcAlias()
endif()

# Build Nanairo
cmake_path(SET nanairo_path "${PROJECT_SOURCE_DIR}/source/nanairo")
add_subdirectory("${nanairo_path}" "${PROJECT_BINARY_DIR}/Nanairo")

## Build examples
#if(ZIVC_BUILD_EXAMPLES)
#  add_subdirectory("${PROJECT_SOURCE_DIR}/example" "${PROJECT_BINARY_DIR}/Examples")
#endif()
#
## Build unit test
#if(ZIVC_BUILD_TESTS)
#  addGoogleTest("${PROJECT_BINARY_DIR}/GoogleTest")
#  enable_testing()
#  add_subdirectory("${PROJECT_SOURCE_DIR}/test" "${PROJECT_BINARY_DIR}/UnitTest")
#endif()
#
#if(PROJECT_IS_TOP_LEVEL)
#  # Build documents
#  add_subdirectory("${PROJECT_SOURCE_DIR}/document" "${PROJECT_BINARY_DIR}/Document")
#
#  # Packaging
#  include("${PROJECT_SOURCE_DIR}/cmake/packaging.cmake")
#  packageProject()
#endif()
