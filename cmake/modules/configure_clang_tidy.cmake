##
# @file configure_clang_tidy.cmake
# @author Bruno Klopott
# @brief Function that bootstraps the clang-tidy static analyser.
#

include(colour_message)

function(configure_clang_tidy enable)

if(${enable})

# Find the necessary executables
find_program(clang_tidy_exe NAMES
  "clang-tidy"
  "clang-tidy-6" "clang-tidy-7" "clang-tidy-8"
  "clang-tidy-6.0" "clang-tidy-7.0" "clang-tidy-8.0"
  "/usr/local/opt/llvm/bin/clang-tidy")
find_program(clang_apply_replacements_exe NAMES
  "clang-apply-replacements"
  "clang-apply-replacements-6"
  "clang-apply-replacements-7"
  "clang-apply-replacements-8"
  "clang-apply-replacements-6.0"
  "clang-apply-replacements-7.0"
  "clang-apply-replacements-8.0"
  "/usr/local/opt/llvm/bin/clang-apply-replacements")
find_program(python NAMES "python" "python2.7")

if (clang_tidy_exe AND clang_apply_replacements_exe)
  message(STATUS "Found clang-tidy: " ${clang_tidy_exe})
  message(STATUS "Found clang-apply-replacements: " ${clang_apply_replacements_exe})
  if (NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  endif ()

  list(GET CMAKE_CXX_CLANG_TIDY 1 checks)
  if (${checks} STREQUAL NOTFOUND)
    set(checks "-checks=-*,-fuchsia*,-clang-diagnostic-unused-parameter,-objc*,llvm-*,bugprone-*,cppcoreguidelines-*,llvm-include-order,clang-analyzer-alpha.*,modernize-*,readability-*,performance-*")
  endif()

  # Enable CLANG_TIDY for each build
  if (${CMAKE_CXX_CLANG_TIDY})
  else ()
    set(CMAKE_CXX_CLANG_TIDY "")
    list(APPEND CMAKE_CXX_CLANG_TIDY ${clang_tidy_exe} ${checks})
  endif ()

  find_program(run_clang_tidy_exe NAMES
    "run-clang-tidy.py" "run_clang_tidy"
    "run-clang-tidy-6.0" "run-clang-tidy-6.0.py")

  if (NOT run_clang_tidy_exe)
    set(run_clang_tidy_exe ${CMAKE_CURRENT_LIST_DIR}/cmake/scripts/run-clang-tidy.py)
  endif ()

  # Provide a global target for linting all library sources
  if(TARGET tidy-all)
  else ()
    add_custom_target(
      tidy-all
      COMMAND "${run_clang_tidy_exe}"
      ${checks}
      -clang-tidy-binary="${clang_tidy_exe}"
      -clang-apply-replacements-binary="${clang_apply_replacements_exe}"
      -extra-arg='-std=c++17'
      -style=Google
      -export-fixes='fixes.yaml'
      COMMENT "Running clang-tidy")
  endif ()
else ()
  colour_message(WARNING RED "Binaries for clang-tidy were not found")
endif()

endif()

endfunction()
