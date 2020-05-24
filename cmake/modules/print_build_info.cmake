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
