include(FindPackageHandleStandardArgs)

file(
	GLOB
	LIBXML2_INCLUDE_PATHS
	$ENV{LIBXML2DIR}/include/libxml2
	$ENV{HOME}/include
	$ENV{HOME}/include/libxml2
	/usr/local/include/libxml2
	/usr/include/libxml2
	$ENV{ProgramW6432}/libxml2*/include
	$ENV{ProgramFiles}/libxml2*/include
)

file(
	GLOB
	LIBXML2_LIBRARY_PATHS
	$ENV{LIBXML2DIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/libxml2*/lib
	$ENV{ProgramFiles}/libxml2*/lib
)

find_path(
	LIBXML2_INCLUDE_DIRS
	NAMES
	libxml/parser.h
	HINTS
	${LIBXML2_INCLUDE_PATHS}
)

find_library(
	LIBXML2_LIBRARIES
	NAMES
	libxml2 xml2
	HINTS
	${LIBXML2_LIBRARY_PATHS}
)

find_package_handle_standard_args(
	LibXml2
	DEFAULT_MSG
	LIBXML2_INCLUDE_DIRS
	LIBXML2_LIBRARIES
)

mark_as_advanced(
	LIBXML2_INCLUDE_DIRS
	LIBXML2_LIBRARIES
) 
