include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	RTAI_INCLUDE_DIR
	NAMES rtai_lxrt.h
	PATHS /usr/realtime/include
	PATH_SUFFIXES rtai
)
find_library(
	RTAI_LIBRARY_RELEASE
	NAMES lxrt
	PATHS /usr/realtime/lib
)
select_library_configurations(RTAI)

if(RTAI_INCLUDE_DIR AND EXISTS "${RTAI_INCLUDE_DIR}/rtai_config.h")
	file(STRINGS "${RTAI_INCLUDE_DIR}/rtai_config.h" _RTAI_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+VERSION[\t ]+\"([^\"]*)\".*" "\\1" RTAI_VERSION "${_RTAI_VERSION_DEFINE}")
	unset(_RTAI_VERSION_DEFINE)
endif()

set(RTAI_INCLUDE_DIRS ${RTAI_INCLUDE_DIR})
set(RTAI_LIBRARIES ${RTAI_LIBRARIES} rt)

find_package_handle_standard_args(
	RTAI
	FOUND_VAR RTAI_FOUND
	REQUIRED_VARS RTAI_INCLUDE_DIR RTAI_LIBRARY
	VERSION_VAR RTAI_VERSION
)

if(RTAI_FOUND AND NOT TARGET RTAI::lxrt)
	add_library(RTAI::lxrt UNKNOWN IMPORTED)
	if(RTAI_LIBRARY_RELEASE)
		set_property(TARGET RTAI::lxrt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(RTAI::lxrt PROPERTIES IMPORTED_LOCATION_RELEASE "${RTAI_LIBRARY_RELEASE}")
	endif()
	set_target_properties(RTAI::lxrt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${RTAI_INCLUDE_DIRS}")
	set_target_properties(RTAI::lxrt PROPERTIES INTERFACE_LINK_LIBRARIES "rt")
endif()

mark_as_advanced(RTAI_INCLUDE_DIR)
