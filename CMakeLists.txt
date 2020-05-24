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
#------------------------------------------------------------------------------
# The build specification for the library
cmake_minimum_required(VERSION 3.6)
project(exot-library VERSION 3.0.0 LANGUAGES CXX)

# Use the following external cmake functions/modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/modules)
include(add_fiber_dependency)
include(colour_message)
include(print_target_properties)
include(print_directory_properties)
include(print_build_info)
include(configure_clang_format)
include(configure_clang_tidy)

# Force CMake to provide compile commands, which are useful for IDEs and static
# analysers.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
get_directory_property(__has_parent PARENT_DIRECTORY)
if(__has_parent)
	set(CMAKE_EXPORT_COMPILE_COMMANDS ON PARENT_SCOPE)
endif()


colour_message(STATUS BLUE BOLD
	"Configuring the exot library ⤵")

# Set of options available for the library
option(exot_use_fibers "Use Boost::fiber" OFF)
option(enable_clang_format "Use code autoformatting" ON)
option(enable_clang_tidy "Use the static analyser" OFF)

if(CMAKE_BUILD_TYPE)
	if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		set(enable_clang_tidy ON)
		# "-Og should be the optimization level of choice for the standard
		# edit-compile-debug cycle, offering a reasonable level of optimization
		# while maintaining fast compilation and a good debugging experience. It
		# is a better choice than -O0 for producing debuggable code because some
		# compiler passes that collect debug information are disabled at -O0."
		# See: https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
		string(APPEND CMAKE_CXX_FLAGS_DEBUG " -Og")
	endif()
endif()

configure_clang_tidy(${enable_clang_tidy})

print_build_info()

#------------------------------------------------------------------------------
# Library

# Add a static library target
add_library(exot STATIC "")
add_library(exot-modules STATIC "")

# CMake versions lower than 3.8 will complain that a 'unknown feature' was
# specified with target_compile_features, and will complain that CXX_STANDARD
# 'contained an invalid value: 17'.
if (${CMAKE_MINOR_VERSION} GREATER 7)
	# A list of used compile features, useful in the case when special arguments
	# are required for the compiler to enable them.
	target_compile_features(exot PUBLIC
		cxx_std_17
		cxx_auto_type
		cxx_constexpr
		cxx_decltype
		cxx_delegating_constructors
		cxx_inline_namespaces
		cxx_lambdas
		cxx_nullptr
		cxx_override
		cxx_range_for
		cxx_rvalue_references
		cxx_static_assert
		cxx_strong_enums
		cxx_template_template_parameters
		cxx_thread_local
		cxx_uniform_initialization
		cxx_variable_templates
		cxx_variadic_templates)

	set_target_properties(exot PROPERTIES
		CXX_STANDARD 17
		CXX_STANDARD_REQUIRED YES)
else ()
	set(CMAKE_CXX11_EXTENSION_COMPILE_OPTION "-std=c++17")
	include(CheckCXXCompilerFlag)
	check_cxx_compiler_flag("-std=c++17" __has_stdcxx_17)
	if (__has_stdcxx_17)
		get_target_property(__exot_compile_options exot COMPILE_OPTIONS)
		if(__exot_compile_options)
			string(REPLACE "-std=c++11" "" __exot_compile_options
				"${__exot_compile_options}")
			string(REPLACE "-std=c++14" "" __exot_compile_options
				"${__exot_compile_options}")
			string(REPLACE "-std=gnu++11" "" __exot_compile_options
				"${__exot_compile_options}")
			string(REPLACE "-std=gnu++14" "" __exot_compile_options
				"${__exot_compile_options}")
			set_target_properties(exot PROPERTIES
				COMPILE_OPTIONS ${__exot_compile_options})
		endif()
		target_compile_options(exot PUBLIC "-std=c++17")
	else()
		message(FATAL_ERROR "The compiler does not support the C++17 standard.")
	endif()
endif ()


# Source and include files
set(include_dir ${CMAKE_CURRENT_LIST_DIR}/include)
set(include_subdir ${include_dir}/exot)

# Core exot library headers & sources
set(framework_headers
	${include_subdir}/framework/all.h
	${include_subdir}/framework/connect.h
	${include_subdir}/framework/executor.h
	${include_subdir}/framework/interface.h
	${include_subdir}/framework/node.h
	${include_subdir}/framework/queue.h
	${include_subdir}/framework/state.h)

set(utilities_headers
	${include_subdir}/utilities/alignment.h
	${include_subdir}/utilities/allocator.h
	${include_subdir}/utilities/barrier.h
	${include_subdir}/utilities/bits.h
	${include_subdir}/utilities/cache.h
	${include_subdir}/utilities/cli.h
	${include_subdir}/utilities/configuration.h
	${include_subdir}/utilities/enum.h
	${include_subdir}/utilities/eviction.h
	${include_subdir}/utilities/filesystem.h
	${include_subdir}/utilities/fmt.h
	${include_subdir}/utilities/formatting.h
	${include_subdir}/utilities/helpers.h
	${include_subdir}/utilities/istream.h
	${include_subdir}/utilities/literals.h
	${include_subdir}/utilities/main.h
	${include_subdir}/utilities/mmap.h
	${include_subdir}/utilities/logging.h
	${include_subdir}/utilities/ostream.h
	${include_subdir}/utilities/pagemap.h
	${include_subdir}/utilities/platform_id.h
	${include_subdir}/utilities/synchronised.h
	${include_subdir}/utilities/thread.h
	${include_subdir}/utilities/timing.h
	${include_subdir}/utilities/timing_source.h
	${include_subdir}/utilities/types.h
	${include_subdir}/utilities/workers.h)

set(primitives_headers
	${include_subdir}/primitives/cache.h
	${include_subdir}/primitives/fenced_clock.h
	${include_subdir}/primitives/monotonic_clock.h
	${include_subdir}/primitives/msr.h
	${include_subdir}/primitives/ordering.h
	${include_subdir}/primitives/perf_clock.h
	${include_subdir}/primitives/rng.h
	${include_subdir}/primitives/tsc.h
	${include_subdir}/primitives/tsc_clock.h
	${include_subdir}/primitives/x86_64.h)

set(components_headers
	${include_subdir}/components/domain_adapter.h
	${include_subdir}/components/function_nodes.h
	${include_subdir}/components/generator_host.h
	${include_subdir}/components/generator_ffb.h
	${include_subdir}/components/logger.h
	${include_subdir}/components/meter_host.h
	${include_subdir}/components/meter_host_logger.h
	${include_subdir}/components/schedule_reader.h)

set(headers "")

list(APPEND headers
	${framework_headers}
	${utilities_headers}
	${primitives_headers}
	${components_headers})

# Modules headers & sources
set(meters_headers
	${include_subdir}/meters/base.h
	${include_subdir}/meters/cache_er.h
	${include_subdir}/meters/cache_ff.h
	${include_subdir}/meters/cache_fp.h
	${include_subdir}/meters/cache_fr.h
	${include_subdir}/meters/cache_l1.h
	${include_subdir}/meters/fan_sysfs.h
	${include_subdir}/meters/fan_procfs.h
	${include_subdir}/meters/frequency.h
	${include_subdir}/meters/frequency_rel.h
	${include_subdir}/meters/frequency_sysfs.h
	${include_subdir}/meters/power.h
	${include_subdir}/meters/power_msr.h
	${include_subdir}/meters/rdseed_status.h
	${include_subdir}/meters/rdseed_timing.h
	${include_subdir}/meters/thermal.h
	${include_subdir}/meters/thermal_msr.h
	${include_subdir}/meters/thermal_sysfs.h
	${include_subdir}/meters/utilisation.h
	${include_subdir}/meters/utilisation_procfs.h
	${include_subdir}/meters/process.h
	${include_subdir}/meters/process_android.h
  )

set(generators_headers
	${include_subdir}/generators/base.h
	${include_subdir}/generators/base_bitset.h
	${include_subdir}/generators/base_ffb.h
	${include_subdir}/generators/base_shared_memory.h
	${include_subdir}/generators/cache_maurice_mt.h
	${include_subdir}/generators/cache_st.h
	${include_subdir}/generators/inactive_mt.h
	${include_subdir}/generators/rdseed_mt.h
	${include_subdir}/generators/utilisation_mt.h
	${include_subdir}/generators/utilisation_ut.h
	${include_subdir}/generators/utilisation_ffb_conservative.h)

set(modules_headers "")
list (APPEND modules_headers
	${meters_headers}
	${generators_headers})

set(source_dir ${CMAKE_CURRENT_LIST_DIR}/src)

set(sources
	${source_dir}/primitives/msr.cpp
	${source_dir}/primitives/x86_64.cpp
	${source_dir}/utilities/barrier.cpp
	${source_dir}/utilities/cli.cpp
	${source_dir}/utilities/configuration.cpp
	${source_dir}/utilities/filesystem.cpp
	${source_dir}/utilities/logging.cpp
	${source_dir}/utilities/platform_id.cpp
	${source_dir}/utilities/thread.cpp
  )

set(modules_sources
	${source_dir}/generators/cache_maurice_mt.cpp
	${source_dir}/generators/rdseed_mt.cpp
	${source_dir}/generators/utilisation_mt.cpp
  ${source_dir}/generators/utilisation_ffb_conservative.cpp
	${source_dir}/generators/utilisation_ut.cpp
	${source_dir}/meters/cache_l1.cpp
	${source_dir}/meters/frequency_rel.cpp
	${source_dir}/meters/frequency_sysfs.cpp
	${source_dir}/meters/power_msr.cpp
	${source_dir}/meters/thermal_msr.cpp
	${source_dir}/meters/thermal_sysfs.cpp
	${source_dir}/meters/utilisation_procfs.cpp
	${source_dir}/meters/process_android.cpp
  )

set(headers_and_sources ${headers} ${sources})
set(modules_headers_and_sources ${modules_headers} ${modules_sources})

#------------------------------------------------------------------------------
# Third-party dependencies

colour_message(STATUS YELLOW BOLD
	"Configuring third-party libraries and dependencies ⤵")

find_package(Threads REQUIRED)

# Import external CMake projects
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/vendor/fmt)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/vendor/spdlog)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/vendor/doctest)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/vendor/json)

# `clipp` is not a CMake project, the include directory is set manually
set(clipp_include_dir ${CMAKE_CURRENT_LIST_DIR}/vendor/clipp/include)
# `doctest` target does not export INCLUDE_DIRECTORIES
set(doctest_include_dir ${CMAKE_CURRENT_LIST_DIR}/vendor/doctest/doctest)

if (${exot_use_fibers})
	find_package(Boost 1.67 COMPONENTS fiber REQUIRED)
endif ()

colour_message(STATUS YELLOW BOLD
	"Finished configuring third-party libraries ⤴")

#------------------------------------------------------------------------------
# Add dependencies and configure the library

target_sources(exot PRIVATE ${headers_and_sources})
target_sources(exot-modules PRIVATE ${modules_headers_and_sources})
target_link_libraries(exot-modules PUBLIC exot)

target_include_directories(exot PUBLIC
	${include_dir}
	${clipp_include_dir})
target_link_libraries(exot PUBLIC
	fmt::fmt
	spdlog
	nlohmann_json::nlohmann_json)
if (${CMAKE_CXX_FLAGS} MATCHES "-static.*")
	message(STATUS "MATCHED: " Threads::Threads)
	target_link_libraries(exot PUBLIC
		"-Wl,--whole-archive" Threads::Threads "-Wl,--no-whole-archive")
else()
	target_link_libraries(exot PUBLIC Threads::Threads)
endif()
if (${ANDROID})
	find_library(android-logging log)
	target_link_libraries(exot PUBLIC ${android-logging})
endif()
add_fiber_dependency(exot)

set_target_properties(exot PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(CMAKE_BUILD_TYPE)
	if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		target_compile_definitions(exot PUBLIC
			-DSPDLOG_DEBUG_ON=1 -DSPDLOG_TRACE_ON=1
			-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
	else()
		target_compile_definitions(exot PUBLIC
			-DSPDLOG_NO_THREAD_ID)
	endif()
endif()

target_compile_options(exot PUBLIC -DSPDLOG_FMT_EXTERNAL)

set(EXOT_TIME_FENCE 0 CACHE STRING "The default timing fence")
set(EXOT_TIME_SOURCE 4 CACHE STRING "The default timing source")
set_property(CACHE EXOT_TIME_FENCE PROPERTY STRINGS 0 1 2 3)
set_property(CACHE EXOT_TIME_SOURCE PROPERTY STRINGS 0 1 2 3 4 5)

if(EXOT_TIME_FENCE STREQUAL 0)
	colour_message(STATUS YELLOW "The default timing serialisation for <exot> is: TimingFenceType::Atomic")
elseif(EXOT_TIME_FENCE STREQUAL 1)
	colour_message(STATUS YELLOW "The default timing serialisation for <exot> is: TimingFenceType::Weak")
elseif(EXOT_TIME_FENCE STREQUAL 2)
	colour_message(STATUS YELLOW "The default timing serialisation for <exot> is: TimingFenceType::Strong")
elseif(EXOT_TIME_FENCE STREQUAL 3)
	colour_message(STATUS YELLOW "The default timing serialisation for <exot> is: TimingFenceType::None")
else()
	colour_message(FATAL_ERROR RED "Invalid default timing serialisation for <exot> chosen: " ${EXOT_TIME_FENCE})
endif()

if(EXOT_TIME_SOURCE STREQUAL 0)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::SteadyClock")
elseif(EXOT_TIME_SOURCE STREQUAL 1)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::MonotonicCounter")
elseif(EXOT_TIME_SOURCE STREQUAL 2)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::MonotonicClock")
elseif(EXOT_TIME_SOURCE STREQUAL 3)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::TimeStampCounter")
elseif(EXOT_TIME_SOURCE STREQUAL 4)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::HardwarePerformanceCounter")
elseif(EXOT_TIME_SOURCE STREQUAL 5)
	colour_message(STATUS YELLOW "The default timing source for <exot> is: TimingSourceType::SoftwarePerformanceCounter")
else()
	colour_message(FATAL_ERROR RED "Invalid default timing source for <exot> chosen: " ${EXOT_TIME_SOURCE})
endif()

target_compile_definitions(exot PUBLIC
	EXOT_TIME_FENCE=${EXOT_TIME_FENCE}
	EXOT_TIME_SOURCE=${EXOT_TIME_SOURCE})

set(GENERATOR_HOST_PERFORM_VALIDATION         0 CACHE STRING "Perform token validation?")
set(GENERATOR_HOST_PROVIDE_TIMING_STATISTICS  1 CACHE STRING "Provide time keeper statistics?")
set(METER_SET_AFFINITY                        1 CACHE STRING "Set meter host affinity?")
set(METER_USE_STATISTICS                      0 CACHE STRING "Provide time keeper statistics?")
set(METER_LOG_SYSTEM_TIME                     0 CACHE STRING "Log system time?")
set(METER_NOW_FROM_TIMER                      1 CACHE STRING "Take timestamp from time keeper?")

set_property(CACHE GENERATOR_HOST_PERFORM_VALIDATION         PROPERTY STRINGS 0 1)
set_property(CACHE GENERATOR_HOST_PROVIDE_TIMING_STATISTICS  PROPERTY STRINGS 0 1)
set_property(CACHE METER_SET_AFFINITY                        PROPERTY STRINGS 0 1)
set_property(CACHE METER_USE_STATISTICS                      PROPERTY STRINGS 0 1)
set_property(CACHE METER_LOG_SYSTEM_TIME                     PROPERTY STRINGS 0 1)
set_property(CACHE METER_NOW_FROM_TIMER                      PROPERTY STRINGS 0 1)

target_compile_definitions(exot PUBLIC
  GENERATOR_HOST_PERFORM_VALIDATION=${GENERATOR_HOST_PERFORM_VALIDATION}
	GENERATOR_HOST_PROVIDE_TIMING_STATISTICS=${GENERATOR_HOST_PROVIDE_TIMING_STATISTICS}
	METER_SET_AFFINITY=${METER_SET_AFFINITY}
	METER_USE_STATISTICS=${METER_USE_STATISTICS}
	METER_LOG_SYSTEM_TIME=${METER_LOG_SYSTEM_TIME}
	METER_NOW_FROM_TIMER=${METER_NOW_FROM_TIMER})

print_target_properties(exot LINK_LIBRARIES)
print_target_properties(exot COMPILE_DEFINITIONS)
print_target_properties(exot COMPILE_OPTIONS)
print_target_properties(exot LINK_FLAGS)
print_target_properties(exot LINK_FLAGS_DEBUG)
print_target_properties(exot LINK_FLAGS_RELEASE)
print_target_properties(exot CXX_STANDARD)
print_target_properties(exot CXX_STANDARD_REQUIRED)

configure_clang_format(${enable_clang_format} exot)

#------------------------------------------------------------------------------
# Testing target

file(GLOB test_sources "${CMAKE_CURRENT_LIST_DIR}/test/test_*.cpp")

add_executable(exot-test ${test_sources})
target_link_libraries(exot-test Threads::Threads exot)
target_include_directories(exot-test PUBLIC ${doctest_include_dir})

colour_message(STATUS BLUE BOLD
	"Finished configuring the exot library ⤴")

#------------------------------------------------------------------------------
# Example code targets

# Joint target for all example apps
add_custom_target(exot-examples)
file(GLOB examples "${CMAKE_CURRENT_LIST_DIR}/examples/*.cpp")

foreach(example ${examples})
	get_filename_component(_name ${example} NAME_WE)
	add_executable(exot-example-${_name} ${example})
	target_link_libraries(exot-example-${_name} exot-modules)
	add_dependencies(exot-examples exot-example-${_name})
endforeach(example)

#------------------------------------------------------------------------------
# Miscellaneous targets from /scratch/misc

add_custom_target(exot-misc)
file(GLOB misc "${CMAKE_CURRENT_LIST_DIR}/scratch/misc/*.cpp")

foreach(example ${misc})
	get_filename_component(_name ${example} NAME_WE)
	add_executable(exot-misc-${_name} ${example})
	if (${_name} MATCHES ".*_no_modules.*")
		target_link_libraries(exot-misc-${_name} exot)
	else ()
		target_link_libraries(exot-misc-${_name} exot-modules)
	endif ()
	add_dependencies(exot-misc exot-misc-${_name})
endforeach(example)

#------------------------------------------------------------------------------
# Temporary app targets from /scratch/apps

add_custom_target(exot-apps)
file(GLOB apps "${CMAKE_CURRENT_LIST_DIR}/scratch/apps/*.cpp")

foreach(example ${apps})
	get_filename_component(_name ${example} NAME_WE)
	add_executable(${_name} ${example})
	if (${_name} MATCHES ".*_no_modules.*")
		target_link_libraries(${_name} exot)
	else ()
		target_link_libraries(${_name} exot-modules)
	endif ()
	add_dependencies(exot-apps ${_name})
endforeach(example)
