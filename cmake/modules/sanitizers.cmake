##
# @file sanitizers.cmake
# @author Bruno Klopott
# @brief Functions to enable sanitisers for compilation targets.
#
# Properties:
# - ENABLED_SANITIZERS
# Functions:
# - target_add_sanitizer: adds a sanitizer runtime to a target
# - target_disable_sanitizers: disables all sanitizers of a target
# - add_sanitizer_target: creates a clone of a target with a sanitizer
# - add_sanitizer_targets: creates clones of a target for all sanitizers

# Sanitizer interactions:
#  1.  leak      + memory:     NOT ALLOWED
#  2.  leak      + thread:     NOT ALLOWED
#  3.  leak      + address:    OK
#  4.  leak      + undefined:  OK
#  5.  memory    + leak:       NOT ALLOWED
#  6.  memory    + thread:     NOT ALLOWED
#  7.  memory    + address:    NOT ALLOWED
#  8.  memory    + undefined:  OK
#  9.  thread    + leak:       NOT ALLOWED
#  10. thread    + memory:     NOT ALLOWED
#  11. thread    + address:    NOT ALLOWED
#  12. thread    + undefined:  OK
#  13. address   + leak:       OK
#  14. address   + memory:     NOT ALLOWED
#  15. address   + thread:     NOT ALLOWED
#  16. address   + undefined:  OK
#  17. undefined + leak:       OK
#  18. undefined + memory:     OK
#  19. undefined + thread:     OK
#  20. undefined + address:    OK

define_property(TARGET PROPERTY ENABLED_SANITIZERS
  BRIEF_DOCS "List of enabled sanitizers for a target."
  FULL_DOCS  "List of enabled sanitizers for a target.
              Has to contain one of: address, memory, leak, thread, undefined.")

include(clone_target)
include(CheckCXXCompilerFlag)

# Adds a sanitizer runtime to a target
function (target_add_sanitizer target sanitizer)
  if (NOT ${sanitizer} MATCHES "(address|memory|leak|thread|undefined)")
    return()
  endif()

  get_target_property(__tmp ${target} ENABLED_SANITIZERS)

  if (__tmp)
      set(__list_of_enabled_sanitizers ${__tmp})
  else()
      set(__list_of_enabled_sanitizers "")
  endif()

  # do not duplicate
  if(${sanitizer} IN_LIST $_list_of_enabled_sanitizers)
    message(WARNING "sanitizer ${sanitizer} already added for target ${target}")
    return()
  endif()

  # the support flags need to be set only once
  if(NOT __list_of_enabled_sanitizers)
    set(__support_flags "-g" "-fno-omit-frame-pointer" "-fno-optimize-sibling-calls")
    check_cxx_compiler_flag("${__support_flags}" __support_flags_compile)
  else()
    set(__support_flags)
  endif()

  #------------------------------------------------------------------ ADDRESS
  if(${sanitizer} STREQUAL "address")
    if("memory" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Address and memory sanitizers cannot be used together. "
        "Address sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("thread" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Address and thread sanitizers cannot be used together. "
        "Address sanitizer will not be added to target '${target}'.")
      return()
    endif()

    list(APPEND __list_of_enabled_sanitizers "address")
    set(__flag "-fsanitize=address")
  #------------------------------------------------------------------ MEMORY
  elseif(${sanitizer} STREQUAL "memory")
    if("address" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Memory and address sanitizers cannot be used together. "
        "Memory sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("leak" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Memory and leak sanitizers cannot be used together. "
        "Memory sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("thread" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Memory and thread sanitizers cannot be used together. "
        "Memory sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
      message(WARNING "Memory sanitizer is not available for the GNU compiler.")
      return()
    endif()

    list(APPEND __list_of_enabled_sanitizers "memory")
    # the memory sanitizer requires compilation with -fPIC set
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(__flag "-fsanitize=memory;-fPIE;-pie")
  #------------------------------------------------------------------ LEAK
  elseif(${sanitizer} STREQUAL "leak")
    if("memory" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Leak and memory sanitizers cannot be used together. "
        "Leak sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("thread" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Leak and thread sanitizers cannot be used together. "
        "Leak sanitizer will not be added to target '${target}'.")
      return()
    endif()

    list(APPEND __list_of_enabled_sanitizers "leak")
    set(__flag "-fsanitize=leak")
  #------------------------------------------------------------------ THREAD
  elseif(${sanitizer} STREQUAL "thread")
    if("address" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Thread and address sanitizers cannot be used together. "
        "Thread sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("memory" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Thread and memory sanitizers cannot be used together. "
        "Thread sanitizer will not be added to target '${target}'.")
      return()
    endif()

    if("leak" IN_LIST __list_of_enabled_sanitizers)
      message(WARNING "Thread and leak sanitizers cannot be used together. "
        "Thread sanitizer will not be added to target '${target}'.")
      return()
    endif()

    list(APPEND __list_of_enabled_sanitizers "thread")
    set(__flag "-fsanitize=thread")
  #------------------------------------------------------------------ UNDEFINED
  elseif(${sanitizer} STREQUAL "undefined")
    list(APPEND __list_of_enabled_sanitizers "undefined")
    set(__flag "-fsanitize=undefined")
  endif()

  message(STATUS "Checking if ${sanitizer} sanitizer flags successfully compile...")

  check_cxx_compiler_flag("${__flag}" __sanitizer_flag_compiles)
  if (__sanitizer_flag_compiles EQUAL 1)
    set(__flags_to_add "${__support_flags}" "${__flag}")
  else ()
    message(WARNING "Compiler check of sanitizer flags failed! "
      "The ${sanitizer} sanitizer will not be added to target '${target}'.")
    return()
  endif()

  set_target_properties(${target} PROPERTIES ENABLED_SANITIZERS "${__list_of_enabled_sanitizers}")
  target_link_libraries(${target} PUBLIC ${__flags_to_add})
  target_compile_options(${target} PUBLIC ${__flags_to_add})

endfunction()

# Disables sanitizers
function (target_disable_sanitizers target)
  target_link_libraries(${target} PUBLIC "-fno-sanitize=all")
  target_compile_options(${target} PUBLIC "-fno-sanitize=all")
endfunction()

# Creates a clone of a target with a sanitizer enabled
function (add_sanitizer_target target sanitizer)
  if (NOT ${sanitizer} MATCHES "(address|memory|leak|thread|undefined)")
    return()
  endif()
  set(new-target "${target}_with_${sanitizer}_sanitizer")

  if (${sanitizer} STREQUAL "memory")
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
      message(WARNING "Memory sanitizer is not available for the GNU compiler."
        "Target ${new-target} will not be created.")
      return()
    endif()
  endif()

  clone_target(${target} ${new-target})
  target_add_sanitizer(${new-target} ${sanitizer})
endfunction()

# Creates a clone of a target for every available sanitizer
function (add_sanitizer_targets target)
  set(sanitizers "address;memory;leak;thread;undefined")
  foreach(sanitizer ${sanitizers})
    add_sanitizer_target(${target} "${sanitizer}")
  endforeach(sanitizer)
endfunction()
