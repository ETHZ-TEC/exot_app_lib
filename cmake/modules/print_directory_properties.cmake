##
# @file print_directory_properties.cmake
# @author Bruno Klopott
# @brief Function that prints a specific property for a given directory.
#

include(colour_message)

function(print_directory_properties property)
	get_directory_property(name SOURCE_DIR)
	get_directory_property(properties ${property})
	string(REPLACE "_" " " property_as_string ${property})
	string(TOLOWER ${property_as_string} property_as_string)

	set(comment "Directory ${property_as_string}:")

	if (properties)
		colour_message(STATUS BLUE ${comment})
		foreach(property ${properties})
			message(STATUS "  " ${property})
		endforeach(property)
	endif ()
endfunction()
