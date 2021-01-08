include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

if(NOT Xenomai_FIND_COMPONENTS)
	set(Xenomai_FIND_COMPONENTS native xenomai)
	foreach(component IN LISTS Xenomai_FIND_COMPONENTS)
		set(Xenomai_FIND_REQUIRED_${component} ON)
	endforeach()
endif()

find_path(
	XENOMAI_INCLUDE_DIR
	NAMES xeno_config.h
	PATHS /usr/xenomai/include
	PATH_SUFFIXES xenomai
)

foreach(component IN LISTS Xenomai_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	find_library(
		XENOMAI_${COMPONENT}_LIBRARY_RELEASE
		NAMES ${component}
		PATHS /usr/xenomai/lib
	)
	select_library_configurations(XENOMAI_${COMPONENT})
endforeach()

if(XENOMAI_INCLUDE_DIR AND EXISTS "${XENOMAI_INCLUDE_DIR}/xeno_config.h")
	file(STRINGS "${XENOMAI_INCLUDE_DIR}/xeno_config.h" _XENOMAI_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+VERSION[\t ]+\"([^\"]*)\".*" "\\1" XENOMAI_VERSION "${_XENOMAI_VERSION_DEFINE}")
	unset(_XENOMAI_VERSION_DEFINE)
endif()

set(XENOMAI_DEFINITIONS -D__XENO__ -D_GNU_SOURCE -D_REENTRANT)
set(XENOMAI_INCLUDE_DIRS ${XENOMAI_INCLUDE_DIR})
set(XENOMAI_LIBRARIES pthread rt)

foreach(component IN LISTS Xenomai_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(XENOMAI_${COMPONENT}_LIBRARY)
		set(Xenomai_${component}_FOUND ON)
		list(APPEND XENOMAI_LIBRARIES ${XENOMAI_${COMPONENT}_LIBRARY})
	endif()
endforeach()

find_package_handle_standard_args(
	Xenomai
	FOUND_VAR Xenomai_FOUND
	REQUIRED_VARS XENOMAI_INCLUDE_DIR
	VERSION_VAR XENOMAI_VERSION
	HANDLE_COMPONENTS
)

foreach(component IN LISTS Xenomai_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(Xenomai_${component}_FOUND AND NOT TARGET Xenomai::${component})
		add_library(Xenomai::${component} UNKNOWN IMPORTED)
		if(XENOMAI_${COMPONENT}_LIBRARY_RELEASE)
			set_property(TARGET Xenomai::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
			set_target_properties(Xenomai::${component} PROPERTIES IMPORTED_LOCATION_RELEASE "${XENOMAI_${COMPONENT}_LIBRARY_RELEASE}")
		endif()
		if(component STREQUAL "xenomai")
			set_target_properties(Xenomai::${component} PROPERTIES INTERFACE_COMPILE_DEFINITIONS "__XENO__;_GNU_SOURCE;_REENTRANT")
		endif()
		set_target_properties(Xenomai::${component} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${XENOMAI_INCLUDE_DIRS}")
		if(component STREQUAL "xenomai")
			set_target_properties(Xenomai::${component} PROPERTIES INTERFACE_LINK_LIBRARIES "pthread;rt")
		endif()
	endif()
endforeach()

mark_as_advanced(XENOMAI_DEFINITIONS)
mark_as_advanced(XENOMAI_INCLUDE_DIR)
