##
# @file configure_clang_format.cmake
# @author Bruno Klopott
# @brief Function that bootstraps clang-format utility.
#

function(configure_clang_format enable target)

if(${enable})

find_program(CLANG_FORMAT NAMES
  "clang-format" "clang-format-6.0" "clang-format-7.0"
  "/usr/local/opt/llvm/bin/clang-format")

if(CLANG_FORMAT)
  message(STATUS "Found clang-format binary: " ${CLANG_FORMAT})

  get_target_property(target_sources ${target} SOURCES)
  get_target_property(target_name ${target} NAME)

  add_custom_target(
    format-${target_name}
    COMMAND ${CLANG_FORMAT}
    -i
    -style=Google
    ${target_sources}
    )
endif()

endif()

endfunction()
