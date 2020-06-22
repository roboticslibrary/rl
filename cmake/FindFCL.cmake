include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	FCL_INCLUDE_DIRS
	NAMES
	fcl/config.h
)

mark_as_advanced(FCL_INCLUDE_DIRS)

find_library(
	FCL_LIBRARY_DEBUG
	NAMES
	fcld
)

find_library(
	FCL_LIBRARY_RELEASE
	NAMES
	fcl
)

select_library_configurations(FCL)

if(FCL_INCLUDE_DIRS AND EXISTS "${FCL_INCLUDE_DIRS}/fcl/config.h")
	file(STRINGS "${FCL_INCLUDE_DIRS}/fcl/config.h" _FCL_VERSION_DEFINE REGEX "#define[\t ]+FCL_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "#define[\t ]+FCL_VERSION[\t ]+\"([^\"]*)\".*" "\\1" FCL_VERSION "${_FCL_VERSION_DEFINE}")
	unset(_FCL_VERSION_DEFINE)
endif()

if(FCL_VERSION AND FCL_VERSION VERSION_LESS 0.5)
	set(FCL_DEFINITIONS -DBOOST_ALL_NO_LIB -DBOOST_SYSTEM_NO_DEPRECATED)
	mark_as_advanced(FCL_DEFINITIONS)
endif()

find_package_handle_standard_args(
	FCL
	FOUND_VAR FCL_FOUND
	REQUIRED_VARS FCL_INCLUDE_DIRS FCL_LIBRARIES
	VERSION_VAR FCL_VERSION
)

if(FCL_FOUND AND NOT TARGET fcl::fcl)
	add_library(fcl::fcl UNKNOWN IMPORTED)
	
	if(FCL_LIBRARY_RELEASE)
		set_property(TARGET fcl::fcl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(fcl::fcl PROPERTIES IMPORTED_LOCATION_RELEASE "${FCL_LIBRARY_RELEASE}")
	endif()
	
	if(FCL_LIBRARY_DEBUG)
		set_property(TARGET fcl::fcl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(fcl::fcl PROPERTIES IMPORTED_LOCATION_DEBUG "${FCL_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		fcl::fcl PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${FCL_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${FCL_INCLUDE_DIRS}"
	)
endif()
