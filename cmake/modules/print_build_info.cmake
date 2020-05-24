# Copyright (c) 2015-2020, Swiss Federal Institute of Technology (ETH Zurich)
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
##
# @file print_build_info.cmake
# @author Bruno Klopott
# @brief Function that prints current build info, including flags, and
#        directory properties.
#

include(colour_message)
include(print_directory_properties)

function(print_build_info)

colour_message(STATUS NONE BOLD "Build info ⤵")

if (DEFINED CMAKE_TOOLCHAIN_FILE)
  get_filename_component(toolchain ${CMAKE_TOOLCHAIN_FILE} NAME)
  message(STATUS "Using toolchain file: " ${Bold}${toolchain}${Reset})
endif ()

message(STATUS "Using Boost::fiber: " ${Bold}${exot_use_fibers}${Reset})
message(STATUS "Exporting compile commands: " ${Bold}${CMAKE_EXPORT_COMPILE_COMMANDS}${Reset})
message(STATUS "Using static analysis: " ${Bold}${enable_clang_tidy}${Reset})
message(STATUS ${Bold} "Build type: " ${Reset} ${CMAKE_BUILD_TYPE})
colour_message(STATUS NONE BOLD "CXX flags:")
message(STATUS ${Bold}           "  Common:        "
	${Reset} ${CMAKE_CXX_FLAGS})
message(STATUS ${Bold}${Red}     "  Debug:          "
	${Reset} ${CMAKE_CXX_FLAGS_DEBUG})
message(STATUS ${Bold}${Green}   "  Release:        "
	${Reset} ${CMAKE_CXX_FLAGS_RELEASE})
message(STATUS ${Bold}${Yellow}  "  RelWithDebInfo: "
	${Reset} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
colour_message(STATUS NONE BOLD "Linker flags:")
message(STATUS ${Bold}           "  Common:        "
  ${Reset} ${CMAKE_EXE_LINKER_FLAGS})
message(STATUS ${Bold}${Red}     "  Debug:          "
  ${Reset} ${CMAKE_EXE_LINKER_FLAGS_DEBUG})
message(STATUS ${Bold}${Green}   "  Release:        "
  ${Reset} ${CMAKE_EXE_LINKER_FLAGS_RELEASE})
message(STATUS ${Bold}${Yellow}  "  RelWithDebInfo: "
  ${Reset} ${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO})

print_directory_properties(COMPILE_DEFINITIONS)
print_directory_properties(COMPILE_OPTIONS)
print_directory_properties(CXX_STANDARD)
print_directory_properties(CXX_STANDARD_REQUIRED)

colour_message(STATUS NONE BOLD "Build info ⤴")

endfunction()
