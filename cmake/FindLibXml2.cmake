include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	LIBXML2_INCLUDE_DIRS
	NAMES
	libxml/parser.h
	PATH_SUFFIXES
	libxml2
)

mark_as_advanced(LIBXML2_INCLUDE_DIRS)

find_library(
	LIBXML2_LIBRARY_DEBUG
	NAMES
	libxml2d xml2d
)
find_library(
	LIBXML2_LIBRARY_RELEASE
	NAMES
	libxml2 xml2
)

select_library_configurations(LIBXML2)

if(LIBXML2_INCLUDE_DIRS AND EXISTS "${LIBXML2_INCLUDE_DIRS}/libxml/xmlversion.h")
	file(STRINGS "${LIBXML2_INCLUDE_DIRS}/libxml/xmlversion.h" _LIBXML2_VERSION_DEFINE REGEX "#define[\t ]+LIBXML_DOTTED_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "#define[\t ]+LIBXML_DOTTED_VERSION[\t ]+\"([^\"]*)\".*" "\\1" LIBXML2_VERSION "${_LIBXML2_VERSION_DEFINE}")
	unset(_LIBXML2_VERSION_DEFINE)
endif()

find_package_handle_standard_args(
	LibXml2
	FOUND_VAR LibXml2_FOUND
	REQUIRED_VARS LIBXML2_INCLUDE_DIRS LIBXML2_LIBRARIES
	VERSION_VAR LIBXML2_VERSION
)

if(LibXml2_FOUND AND NOT TARGET LibXml2::LibXml2)
	add_library(LibXml2::LibXml2 UNKNOWN IMPORTED)
	
	if(LIBXML2_LIBRARY_RELEASE)
		set_property(TARGET LibXml2::LibXml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(LibXml2::LibXml2 PROPERTIES IMPORTED_LOCATION_RELEASE "${LIBXML2_LIBRARY_RELEASE}")
	endif()
	
	if(LIBXML2_LIBRARY_DEBUG)
		set_property(TARGET LibXml2::LibXml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(LibXml2::LibXml2 PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBXML2_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		LibXml2::LibXml2 PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${LIBXML2_INCLUDE_DIRS}"
	)
endif()
