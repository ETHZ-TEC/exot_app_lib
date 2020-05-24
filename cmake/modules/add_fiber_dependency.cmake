##
# @file add_fiber_dependency.cmake
# @author Bruno Klopott
# @brief Function to add a boost::fiber dependency to a cmake target.
#

# If EXOT_USE_FIBERS is set during CMake configuration step, link the
# appropriate library, provide include directories, and
# add a compile definition (equivalent to setting a preprocessor value)
# to the selected target.
function(add_fiber_dependency target)
	if(${exot_use_fibers})
		find_package(Boost 1.67 COMPONENTS fiber REQUIRED)
		target_link_libraries(${target} PUBLIC Boost::fiber)
		target_include_directories(${target} PUBLIC Boost::fiber)
		target_compile_definitions(${target} PUBLIC "EXOT_USE_FIBERS")
	endif(${exot_use_fibers})
endfunction()
