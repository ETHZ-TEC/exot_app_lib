##
# @file print_target_properties.cmake
# @author Bruno Klopott
# @brief Function that prints a specific property for a given target.
#

include(colour_message)

function(print_target_properties target property)
	get_target_property(name ${target} NAME)
	get_target_property(properties ${target} ${property})
	string(REPLACE "_" " " property_as_string ${property})
	string(TOLOWER ${property_as_string} property_as_string)

	set(comment "${name} ${property_as_string}:")

	if (properties)
		colour_message(STATUS BLUE ${comment})
		foreach(property ${properties})
			message(STATUS "  " ${property})
		endforeach(property)
	endif ()
endfunction()
