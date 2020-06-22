include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	LIBXSLT_INCLUDE_DIRS
	NAMES
	libxslt/xslt.h
)

mark_as_advanced(LIBXSLT_INCLUDE_DIRS)

find_library(
	LIBXSLT_LIBRARY_DEBUG
	NAMES
	libxsltd xsltd
)
find_library(
	LIBXSLT_LIBRARY_RELEASE
	NAMES
	libxslt xslt
)

select_library_configurations(LIBXSLT)

if(LIBXSLT_INCLUDE_DIRS AND EXISTS "${LIBXSLT_INCLUDE_DIRS}/libxslt/xsltconfig.h")
	file(STRINGS "${LIBXSLT_INCLUDE_DIRS}/libxslt/xsltconfig.h" _LIBXSLT_VERSION_DEFINE REGEX "#define[\t ]+LIBXSLT_DOTTED_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "#define[\t ]+LIBXSLT_DOTTED_VERSION[\t ]+\"([^\"]*)\".*" "\\1" LIBXSLT_VERSION "${_LIBXSLT_VERSION_DEFINE}")
	unset(_LIBXSLT_VERSION_DEFINE)
endif()

find_package_handle_standard_args(
	LibXslt
	FOUND_VAR LIBXSLT_FOUND
	REQUIRED_VARS LIBXSLT_INCLUDE_DIRS LIBXSLT_LIBRARIES
	VERSION_VAR LIBXSLT_VERSION
)

if(LIBXSLT_FOUND AND NOT TARGET LibXslt::LibXslt)
	add_library(LibXslt::LibXslt UNKNOWN IMPORTED)
	
	if(LIBXSLT_LIBRARY_RELEASE)
		set_property(TARGET LibXslt::LibXslt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(LibXslt::LibXslt PROPERTIES IMPORTED_LOCATION_RELEASE "${LIBXSLT_LIBRARY_RELEASE}")
	endif()
	
	if(LIBXSLT_LIBRARY_DEBUG)
		set_property(TARGET LibXslt::LibXslt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(LibXslt::LibXslt PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBXSLT_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		LibXslt::LibXslt PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${LIBXSLT_INCLUDE_DIRS}"
	)
endif()
