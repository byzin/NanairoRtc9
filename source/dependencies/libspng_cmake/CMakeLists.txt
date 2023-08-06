# file: CMakeLists.txt
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
#

cmake_minimum_required(VERSION 3.26)


#
macro(Nanairo_setSpngProject)
  set(project_description "Simple, modern libpng alternative")
  project(spng VERSION 0.7.4 DESCRIPTION "${project_description}" LANGUAGES C)

  cmake_path(SET zivc_path NORMALIZE "${PROJECT_SOURCE_DIR}/../Zivc/source/zivc")
  include("${zivc_path}/cmake/general.cmake")
  #
  Zivc_checkTarget(Zlib::zlib)

  cmake_path(SET spng_dir NORMALIZE "${PROJECT_SOURCE_DIR}/../libspng")
  if(NOT CMAKE_HOST_WIN32)
    set(math_library "m")
  else()
    set(math_library "")
  endif()

  set(spng_source_files ${spng_dir}/spng/spng.h ${spng_dir}/spng/spng.c)
  source_group(${PROJECT_NAME} FILES ${spng_source_files})
  add_library(${PROJECT_NAME} STATIC ${spng_source_files})
  #
  target_compile_definitions(${PROJECT_NAME} PUBLIC SPNG_STATIC=1
                                                    SPNG_DISABLE_OPT=1)
  target_link_libraries(${PROJECT_NAME} PUBLIC Zlib::zlib
                                        ${math_library})
  target_include_directories(${PROJECT_NAME} INTERFACE ${spng_dir})
endmacro(Nanairo_setSpngProject)


#
Nanairo_setSpngProject()
