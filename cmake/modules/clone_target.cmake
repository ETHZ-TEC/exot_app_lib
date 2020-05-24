##
# @file clone_target.cmake
# @author Bruno Klopott
# @brief Function to clone a target 'as completely as possible'.
#

# list of possible target properties, filtered for read-only and
# incompatible ones.
set(POSSIBLE_TARGET_PROPERTIES
    "ALIASED_TARGET"
    "ANDROID_ANT_ADDITIONAL_OPTIONS"
    "ANDROID_API"
    "ANDROID_API_MIN"
    "ANDROID_ARCH"
    "ANDROID_ASSETS_DIRECTORIES"
    "ANDROID_GUI"
    "ANDROID_JAR_DEPENDENCIES"
    "ANDROID_JAR_DIRECTORIES"
    "ANDROID_JAVA_SOURCE_DIR"
    "ANDROID_NATIVE_LIB_DEPENDENCIES"
    "ANDROID_NATIVE_LIB_DIRECTORIES"
    "ANDROID_PROCESS_MAX"
    "ANDROID_PROGUARD"
    "ANDROID_PROGUARD_CONFIG_PATH"
    "ANDROID_SECURE_PROPS_PATH"
    "ANDROID_SKIP_ANT_STEP"
    "ANDROID_STL_TYPE"
    "ARCHIVE_OUTPUT_DIRECTORY_DEBUG"
    "ARCHIVE_OUTPUT_DIRECTORY_RELEASE"
    "ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO"
    "ARCHIVE_OUTPUT_DIRECTORY"
    "ARCHIVE_OUTPUT_NAME_DEBUG"
    "ARCHIVE_OUTPUT_NAME_RELEASE"
    "ARCHIVE_OUTPUT_NAME_RELWITHDEBINFO"
    "ARCHIVE_OUTPUT_NAME"
    "AUTOGEN_BUILD_DIR"
    "AUTOGEN_PARALLEL"
    "AUTOGEN_TARGET_DEPENDS"
    "AUTOMOC_COMPILER_PREDEFINES"
    "AUTOMOC_DEPEND_FILTERS"
    "AUTOMOC_MACRO_NAMES"
    "AUTOMOC_MOC_OPTIONS"
    "AUTOMOC"
    "AUTOUIC"
    "AUTOUIC_OPTIONS"
    "AUTOUIC_SEARCH_PATHS"
    "AUTORCC"
    "AUTORCC_OPTIONS"
    "BINARY_DIR"
    "BUILD_RPATH"
    "BUILD_WITH_INSTALL_NAME_DIR"
    "BUILD_WITH_INSTALL_RPATH"
    "BUNDLE_EXTENSION"
    "BUNDLE"
    "C_EXTENSIONS"
    "C_STANDARD"
    "C_STANDARD_REQUIRED"
    "COMMON_LANGUAGE_RUNTIME"
    "COMPATIBLE_INTERFACE_BOOL"
    "COMPATIBLE_INTERFACE_NUMBER_MAX"
    "COMPATIBLE_INTERFACE_NUMBER_MIN"
    "COMPATIBLE_INTERFACE_STRING"
    "COMPILE_DEFINITIONS"
    "COMPILE_FEATURES"
    "COMPILE_FLAGS"
    "COMPILE_OPTIONS"
    "COMPILE_PDB_NAME"
    "COMPILE_PDB_NAME_DEBUG"
    "COMPILE_PDB_NAME_RELEASE"
    "COMPILE_PDB_NAME_RELWITHDEBINFO"
    "COMPILE_PDB_OUTPUT_DIRECTORY"
    "COMPILE_PDB_OUTPUT_DIRECTORY_DEBUG"
    "COMPILE_PDB_OUTPUT_DIRECTORY_RELEASE"
    "COMPILE_PDB_OUTPUT_DIRECTORY_RELWITHDEBINFO"
    "CROSSCOMPILING_EMULATOR"
    "CUDA_SEPARABLE_COMPILATION"
    "CUDA_RESOLVE_DEVICE_SYMBOLS"
    "CUDA_EXTENSIONS"
    "CUDA_STANDARD"
    "CUDA_STANDARD_REQUIRED"
    "CXX_EXTENSIONS"
    "CXX_STANDARD"
    "CXX_STANDARD_REQUIRED"
    "DEBUG_POSTFIX"
    "DEFINE_SYMBOL"
    "DEPLOYMENT_REMOTE_DIRECTORY"
    "DOTNET_TARGET_FRAMEWORK_VERSION"
    "EchoString"
    "ENABLE_EXPORTS"
    "ENABLED_SANITIZERS"
    "EXCLUDE_FROM_ALL"
    "EXCLUDE_FROM_DEFAULT_BUILD_DEBUG"
    "EXCLUDE_FROM_DEFAULT_BUILD_RELEASE"
    "EXCLUDE_FROM_DEFAULT_BUILD_RELWITHDEBINFO"
    "EXCLUDE_FROM_DEFAULT_BUILD"
    "EXPORT_NAME"
    "EXPORT_PROPERTIES"
    "FOLDER"
    "Fortran_FORMAT"
    "Fortran_MODULE_DIRECTORY"
    "FRAMEWORK"
    "FRAMEWORK_VERSION"
    "GENERATOR_FILE_NAME"
    "GNUtoMS"
    "HAS_CXX"
    "IMPLICIT_DEPENDS_INCLUDE_TRANSFORM"
    "IMPORT_PREFIX"
    "IMPORT_SUFFIX"
    "INCLUDE_DIRECTORIES"
    "INSTALL_NAME_DIR"
    "INSTALL_RPATH"
    "INSTALL_RPATH_USE_LINK_PATH"
    "INTERPROCEDURAL_OPTIMIZATION_DEBUG"
    "INTERPROCEDURAL_OPTIMIZATION_RELEASE"
    "INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO"
    "INTERPROCEDURAL_OPTIMIZATION"
    "IOS_INSTALL_COMBINED"
    "JOB_POOL_COMPILE"
    "JOB_POOL_LINK"
    "LABELS"
    "C_CLANG_TIDY"
    "CXX_CLANG_TIDY"
    "C_COMPILER_LAUNCHER"
    "CXX_COMPILER_LAUNCHER"
    "C_CPPCHECK"
    "CXX_CPPCHECK"
    "C_CPPLINT"
    "CXX_CPPLINT"
    "C_INCLUDE_WHAT_YOU_USE"
    "CXX_INCLUDE_WHAT_YOU_USE"
    "C_VISIBILITY_PRESET"
    "CXX_VISIBILITY_PRESET"
    "LIBRARY_OUTPUT_DIRECTORY_DEBUG"
    "LIBRARY_OUTPUT_DIRECTORY_RELEASE"
    "LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO"
    "LIBRARY_OUTPUT_DIRECTORY"
    "LIBRARY_OUTPUT_NAME_DEBUG"
    "LIBRARY_OUTPUT_NAME_RELEASE"
    "LIBRARY_OUTPUT_NAME_RELWITHDEBINFO"
    "LIBRARY_OUTPUT_NAME"
    "LINK_DEPENDS_NO_SHARED"
    "LINK_DEPENDS"
    "LINKER_LANGUAGE"
    "LINK_FLAGS_DEBUG"
    "LINK_FLAGS_RELEASE"
    "LINK_FLAGS_RELWITHDEBINFO"
    "LINK_FLAGS"
    "LINK_INTERFACE_LIBRARIES_DEBUG"
    "LINK_INTERFACE_LIBRARIES_RELEASE"
    "LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO"
    "LINK_INTERFACE_LIBRARIES"
    "LINK_INTERFACE_MULTIPLICITY_DEBUG"
    "LINK_INTERFACE_MULTIPLICITY_RELEASE"
    "LINK_INTERFACE_MULTIPLICITY_RELWITHDEBINFO"
    "LINK_INTERFACE_MULTIPLICITY"
    "LINK_LIBRARIES"
    "LINK_SEARCH_END_STATIC"
    "LINK_SEARCH_START_STATIC"
    "LINK_WHAT_YOU_USE"
    "MACOSX_BUNDLE_INFO_PLIST"
    "MACOSX_BUNDLE"
    "MACOSX_FRAMEWORK_INFO_PLIST"
    "MACOSX_RPATH"
    "MAP_IMPORTED_CONFIG_DEBUG"
    "MAP_IMPORTED_CONFIG_RELEASE"
    "MAP_IMPORTED_CONFIG_RELWITHDEBINFO"
    "NO_SONAME"
    "NO_SYSTEM_FROM_IMPORTED"
    "OSX_ARCHITECTURES_DEBUG"
    "OSX_ARCHITECTURES_RELEASE"
    "OSX_ARCHITECTURES_RELWITHDEBINFO"
    "OSX_ARCHITECTURES"
    "OUTPUT_NAME_DEBUG"
    "OUTPUT_NAME_RELEASE"
    "OUTPUT_NAME_RELWITHDEBINFO"
    "OUTPUT_NAME"
    "PDB_NAME_DEBUG"
    "PDB_NAME_RELEASE"
    "PDB_NAME_RELWITHDEBINFO"
    "PDB_NAME"
    "PDB_OUTPUT_DIRECTORY_DEBUG"
    "PDB_OUTPUT_DIRECTORY_RELEASE"
    "PDB_OUTPUT_DIRECTORY_RELWITHDEBINFO"
    "PDB_OUTPUT_DIRECTORY"
    "POSITION_INDEPENDENT_CODE"
    "PREFIX"
    "PRIVATE_HEADER"
    "PROJECT_LABEL"
    "PUBLIC_HEADER"
    "RESOURCE"
    "RULE_LAUNCH_COMPILE"
    "RULE_LAUNCH_CUSTOM"
    "RULE_LAUNCH_LINK"
    "RUNTIME_OUTPUT_DIRECTORY_DEBUG"
    "RUNTIME_OUTPUT_DIRECTORY_RELEASE"
    "RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO"
    "RUNTIME_OUTPUT_DIRECTORY"
    "RUNTIME_OUTPUT_NAME_DEBUG"
    "RUNTIME_OUTPUT_NAME_RELEASE"
    "RUNTIME_OUTPUT_NAME_RELWITHDEBINFO"
    "RUNTIME_OUTPUT_NAME"
    "SKIP_BUILD_RPATH"
    "SOURCE_DIR"
    "SOURCES"
    "SOVERSION"
    "STATIC_LIBRARY_FLAGS_DEBUG"
    "STATIC_LIBRARY_FLAGS_RELEASE"
    "STATIC_LIBRARY_FLAGS_RELWITHDEBINFO"
    "STATIC_LIBRARY_FLAGS"
    "SUFFIX"
    "VERSION")

# Creates a new target of same type and copies all possible properties
function (clone_target target new-target)
    get_target_property(__type ${target} "TYPE")

    if (__type STREQUAL "EXECUTABLE")
        add_executable(${new-target} "")
    elseif (__type STREQUAL "STATIC_LIBRARY")
        add_library(${new-target} STATIC "")
    elseif (__type STREQUAL "SHARED_LIBRARY")
        add_library(${new-target} SHARED "")
    else()
        # TODO: Unsure how to dermine whether a property is only meant
        # for, e.g., iterface libraries.
        message(WARNING "target type ${__type} is not supported!")
        return()
    endif()

    message(STATUS "Created a new target of type ${__type}")
    message(STATUS "Copying target properties from ${target} to ${new-target}")

    # Copy target properties to the newly created target
    foreach(property ${POSSIBLE_TARGET_PROPERTIES})
        get_target_property(__property ${target} ${property})

        if(__property)
            set_target_properties(${new-target} PROPERTIES "${property}" "${__property}")
            message(STATUS "Set property ${property} to ${__property}")
        endif()
    endforeach(property)
endfunction()
