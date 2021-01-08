include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	ICU_INCLUDE_DIR
	NAMES unicode/utypes.h
)

foreach(component IN LISTS ICU_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	set(names_debug icu${component}d sicu${component}d)
	set(names_release icu${component} sicu${component})
	if(component STREQUAL "data")
		list(APPEND names_debug icudtd sicudtd)
		list(APPEND names_release icudt sicudt)
	endif()
	if(component STREQUAL "dt")
		list(APPEND names_debug icudatad sicudatad)
		list(APPEND names_release icudata sicudata)
	endif()
	if(component STREQUAL "i18n")
		list(APPEND names_debug icuind sicuind)
		list(APPEND names_release icuin sicuin)
	endif()
	if(component STREQUAL "in")
		list(APPEND names_debug icui18nd sicui18nd)
		list(APPEND names_release icui18n sicui18n)
	endif()
	if(APPLE)
		list(APPEND names_debug icucored)
		list(APPEND names_release icucore)
	endif()
	find_library(
		ICU_${COMPONENT}_LIBRARY_DEBUG
		NAMES ${names_debug}
	)
	find_library(
		ICU_${COMPONENT}_LIBRARY_RELEASE
		NAMES ${names_release}
	)
	select_library_configurations(ICU_${COMPONENT})
endforeach()

if(ICU_INCLUDE_DIR AND EXISTS "${ICU_INCLUDE_DIR}/unicode/uvernum.h")
	file(STRINGS "${ICU_INCLUDE_DIR}/unicode/uvernum.h" _ICU_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+U_ICU_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+U_ICU_VERSION[\t ]+\"([^\"]*)\".*" "\\1" ICU_VERSION "${_ICU_VERSION_DEFINE}")
	unset(_ICU_VERSION_DEFINE)
endif()

set(ICU_INCLUDE_DIRS ${ICU_INCLUDE_DIR})
unset(ICU_LIBRARIES)

foreach(component IN LISTS ICU_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(ICU_${COMPONENT}_LIBRARY)
		set(ICU_${component}_FOUND ON)
		set(ICU_${COMPONENT}_FOUND ON)
		list(APPEND ICU_LIBRARIES ${ICU_${COMPONENT}_LIBRARY})
	endif()
endforeach()

if(APPLE)
	list(REMOVE_DUPLICATES ICU_LIBRARIES)
endif()

if(APPLE AND NOT ICU_INCLUDE_DIR)
	find_package_handle_standard_args(
		ICU
		FOUND_VAR ICU_FOUND
		REQUIRED_VARS ICU_LIBRARIES
	)
else()
	find_package_handle_standard_args(
		ICU
		FOUND_VAR ICU_FOUND
		REQUIRED_VARS ICU_INCLUDE_DIR
		VERSION_VAR ICU_VERSION
		HANDLE_COMPONENTS
	)
endif()

foreach(component IN LISTS ICU_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(ICU_${component}_FOUND AND NOT TARGET ICU::${component})
		add_library(ICU::${component} UNKNOWN IMPORTED)
		if(ICU_${COMPONENT}_LIBRARY_RELEASE)
			set_property(TARGET ICU::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
			set_target_properties(ICU::${component} PROPERTIES IMPORTED_LOCATION_RELEASE "${ICU_${COMPONENT}_LIBRARY_RELEASE}")
		endif()
		if(ICU_${COMPONENT}_LIBRARY_DEBUG)
			set_property(TARGET ICU::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
			set_target_properties(ICU::${component} PROPERTIES IMPORTED_LOCATION_RELEASE "${ICU_${COMPONENT}_LIBRARY_DEBUG}")
		endif()
		if(ICU_INCLUDE_DIRS)
			set_target_properties(ICU::${component} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ICU_INCLUDE_DIRS}")
		endif()
	endif()
endforeach()

mark_as_advanced(ICU_INCLUDE_DIR)
