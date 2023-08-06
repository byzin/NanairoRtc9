# file: packaging.cmake
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
#


function(packageProject)
  include(InstallRequiredSystemLibraries)

  cmake_path(SET package_directory "${PROJECT_BINARY_DIR}/Packaging")
  set(CPACK_PACKAGE_DIRECTORY "${package_directory}")

  # Set project info
  set(CPACK_PACKAGE_NAME "Nanairo")
  set(CPACK_PACKAGE_VENDOR "Sho Ikeda")
  #  set(CPACK_PACKAGE_VERSION_MAJOR ${Nanairo_VERSION_MAJOR})
  #  set(CPACK_PACKAGE_VERSION_MINOR ${Nanairo_VERSION_MINOR})
  #  set(CPACK_PACKAGE_VERSION_PATCH ${Nanairo_VERSION_PATCH})
  #  set(CPACK_PACKAGE_DESCRIPTION ${Nanairo_DESCRIPTION})
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The path tracing renderer for RTC9.")
  set(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.md)
  set(CPACK_THREADS 0)

  include(CPack)
endfunction(packageProject)
