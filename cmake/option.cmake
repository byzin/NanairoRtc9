# file: option.cmake
# author: Sho Ikeda
#
# Copyright (c) 2015-2023 Sho Ikeda
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
#


# Validate options
function(validateOptions)
endfunction(validateOptions)


# Set command options
function(initProjectOptions)
  cmake_path(SET dependency_path "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../source/dependencies")
  include("${dependency_path}/Zivc/source/zivc/cmake/general.cmake")

  set(option_description "Suppress excessive warnings.")
  Zivc_setBooleanOption(ZIVC_SUPPRESS_EXCESSIVE_WARNING ON ${option_description})

  set(option_description "Enable Vulkan backend.")
  Zivc_setBooleanOption(Z_ENABLE_VULKAN_BACKEND ON ${option_description})

  validateOptions()
endfunction(initProjectOptions)
