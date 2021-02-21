include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	OCTOMAP_INCLUDE_DIR
	NAMES octomap/octomap.h
)

foreach(component IN ITEMS octomath octomap)
	string(TOUPPER ${component} COMPONENT)
	find_library(
		OCTOMAP_${COMPONENT}_LIBRARY_DEBUG
		NAMES ${component}d
	)
	find_library(
		OCTOMAP_${COMPONENT}_LIBRARY_RELEASE
		NAMES ${component}
	)
	select_library_configurations(OCTOMAP_${COMPONENT})
endforeach()

find_file(
	OCTOMAP_PACKAGE_FILE
	NAMES octomap/package.xml octomap/stack.xml
	PATH_SUFFIXES share
)

if(OCTOMAP_PACKAGE_FILE)
	file(STRINGS "${OCTOMAP_PACKAGE_FILE}" _OCTOMAP_VERSION_TAG REGEX ".*<version>[^<]*</version>.*")
	string(REGEX REPLACE ".*<version>([^<]*)</version>.*" "\\1" OCTOMAP_VERSION "${_OCTOMAP_VERSION_TAG}")
	unset(_OCTOMAP_VERSION_TAG)
endif()

set(OCTOMAP_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIR})
set(OCTOMAP_LIBRARIES ${OCTOMAP_OCTOMATH_LIBRARY} ${OCTOMAP_OCTOMAP_LIBRARY})

foreach(component IN ITEMS octomath octomap)
	string(TOUPPER ${component} COMPONENT)
	if(OCTOMAP_${COMPONENT}_LIBRARY)
		set(octomap_${component}_FOUND ON)
	endif()
endforeach()

find_package_handle_standard_args(
	octomap
	FOUND_VAR octomap_FOUND
	REQUIRED_VARS OCTOMAP_INCLUDE_DIR OCTOMAP_OCTOMATH_LIBRARY OCTOMAP_OCTOMAP_LIBRARY
	VERSION_VAR OCTOMAP_VERSION
)

foreach(component IN ITEMS octomath octomap)
	string(TOUPPER ${component} COMPONENT)
	if(octomap_${component}_FOUND AND NOT TARGET octomap::${component})
		add_library(octomap::${component} UNKNOWN IMPORTED)
		if(OCTOMAP_${COMPONENT}_LIBRARY_RELEASE)
			set_property(TARGET octomap::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
			set_target_properties(octomap::${component} PROPERTIES IMPORTED_LOCATION_RELEASE "${OCTOMAP_${COMPONENT}_LIBRARY_RELEASE}")
		endif()
		if(OCTOMAP_${COMPONENT}_LIBRARY_DEBUG)
			set_property(TARGET octomap::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
			set_target_properties(octomap::${component} PROPERTIES IMPORTED_LOCATION_DEBUG "${OCTOMAP_${COMPONENT}_LIBRARY_DEBUG}")
		endif()
		set_target_properties(octomap::${component} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
		if(component STREQUAL "octomap" AND TARGET octomap::octomath)
			set_target_properties(octomap::${component} PROPERTIES INTERFACE_LINK_LIBRARIES "octomap::octomath")
		endif()
	endif()
endforeach()

mark_as_advanced(OCTOMAP_INCLUDE_DIR)
mark_as_advanced(OCTOMAP_PACKAGE_FILE)
