include(FindPackageHandleStandardArgs)

file(
	GLOB
	ZLIB_INCLUDE_PATHS
	$ENV{ZLIBDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/zlib*/include
	$ENV{ProgramFiles}/zlib*/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
)

file(
	GLOB
	ZLIB_LIBRARY_PATHS
	$ENV{ZLIBDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/zlib*/lib
	$ENV{ProgramFiles}/zlib*/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_path(
	ZLIB_INCLUDE_DIRS
	NAMES
	zlib.h
	HINTS
	${ZLIB_INCLUDE_PATHS}
)

find_library(
	ZLIB_LIBRARIES
	NAMES
	z zlib zlib_a zlibstat zlibwapi zdll
	HINTS
	${ZLIB_LIBRARY_PATHS}
)

find_package_handle_standard_args(
	ZLIB
	DEFAULT_MSG
	ZLIB_INCLUDE_DIRS
	ZLIB_LIBRARIES
)

mark_as_advanced(
	ZLIB_INCLUDE_DIRS
	ZLIB_LIBRARIES
) 
