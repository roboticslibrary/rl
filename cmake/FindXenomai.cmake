include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	Xenomai_INCLUDE_DIRS
	NAMES
	native/task.h
	PATHS
	/usr/xenomai/include
	PATH_SUFFIXES
	xenomai
)

mark_as_advanced(Xenomai_INCLUDE_DIRS)

find_library(
	Xenomai_NATIVE_LIBRARY_DEBUG
	NAMES
	natived
	PATHS
	/usr/xenomai/lib
)

find_library(
	Xenomai_NATIVE_LIBRARY_RELEASE
	NAMES
	native
	PATHS
	/usr/xenomai/lib
)

select_library_configurations(Xenomai_NATIVE)

find_library(
	Xenomai_XENOMAI_LIBRARY_DEBUG
	NAMES
	xenomaid
	PATHS
	/usr/xenomai/lib
)

find_library(
	Xenomai_XENOMAI_LIBRARY_RELEASE
	NAMES
	xenomai
	PATHS
	/usr/xenomai/lib
)

select_library_configurations(Xenomai_XENOMAI)

set(Xenomai_LIBRARIES ${Xenomai_NATIVE_LIBRARIES} ${Xenomai_XENOMAI_LIBRARIES} pthread rt)

set(Xenomai_DEFINITIONS -D__XENO__ -D_GNU_SOURCE -D_REENTRANT)

mark_as_advanced(Xenomai_DEFINITIONS)

find_package_handle_standard_args(
	Xenomai
	FOUND_VAR Xenomai_FOUND
	REQUIRED_VARS Xenomai_INCLUDE_DIRS Xenomai_NATIVE_LIBRARY Xenomai_XENOMAI_LIBRARY
)

if(Xenomai_FOUND AND NOT TARGET Xenomai::native)
	add_library(Xenomai::native UNKNOWN IMPORTED)
	
	if(Xenomai_NATIVE_LIBRARY_RELEASE)
		set_property(TARGET Xenomai::native APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Xenomai::native PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_NATIVE_LIBRARY_RELEASE}")
	endif()
	
	if(Xenomai_LIBRARY_DEBUG)
		set_property(TARGET Xenomai::native APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Xenomai::native PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_NATIVE_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Xenomai::native PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Xenomai_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Xenomai_INCLUDE_DIRS}"
	)
endif()

if(Xenomai_FOUND AND NOT TARGET Xenomai::xenomai)
	add_library(Xenomai::xenomai UNKNOWN IMPORTED)
	
	if(Xenomai_LIBRARY_RELEASE)
		set_property(TARGET Xenomai::xenomai APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_XENOMAI_LIBRARY_RELEASE}")
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "Xenomai::native")
	endif()
	
	if(Xenomai_LIBRARY_DEBUG)
		set_property(TARGET Xenomai::xenomai APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LOCATION_DEBUG "${Xenomai_XENOMAI_LIBRARY_DEBUG}")
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "Xenomai::native")
	endif()
	
	set_target_properties(
		Xenomai::xenomai PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Xenomai_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Xenomai_INCLUDE_DIRS}"
	)
endif()
