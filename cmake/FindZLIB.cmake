include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	ZLIB_INCLUDE_DIR
	NAMES zlib.h
)
find_library(
	ZLIB_LIBRARY_DEBUG
	NAMES zd zlibd zdlld zlib1d zlib_ad zlibstaticd zlibwapid
)
find_library(
	ZLIB_LIBRARY_RELEASE
	NAMES z zlib zdll zlib1 zlib_a zlibstatic zlibwapi
)
select_library_configurations(ZLIB)

if(ZLIB_INCLUDE_DIR AND EXISTS "${ZLIB_INCLUDE_DIR}/zlib.h")
	file(STRINGS "${ZLIB_INCLUDE_DIR}/zlib.h" _ZLIB_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+ZLIB_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+ZLIB_VERSION[\t ]+\"([^\"]*)\".*" "\\1" ZLIB_VERSION "${_ZLIB_VERSION_DEFINE}")
	unset(_ZLIB_VERSION_DEFINE)
endif()

set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
set(ZLIB_LIBRARIES ${ZLIB_LIBRARY})

find_package_handle_standard_args(
	ZLIB
	FOUND_VAR ZLIB_FOUND
	REQUIRED_VARS ZLIB_INCLUDE_DIR ZLIB_LIBRARY
	VERSION_VAR ZLIB_VERSION
)

if(ZLIB_FOUND AND NOT TARGET ZLIB::ZLIB)
	add_library(ZLIB::ZLIB UNKNOWN IMPORTED)
	if(ZLIB_LIBRARY_RELEASE)
		set_property(TARGET ZLIB::ZLIB APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(ZLIB::ZLIB PROPERTIES IMPORTED_LOCATION_RELEASE "${ZLIB_LIBRARY_RELEASE}")
	endif()
	if(ZLIB_LIBRARY_DEBUG)
		set_property(TARGET ZLIB::ZLIB APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(ZLIB::ZLIB PROPERTIES IMPORTED_LOCATION_DEBUG "${ZLIB_LIBRARY_DEBUG}")
	endif()
	set_target_properties(ZLIB::ZLIB PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ZLIB_INCLUDE_DIRS}")
endif()

mark_as_advanced(ZLIB_INCLUDE_DIR)
