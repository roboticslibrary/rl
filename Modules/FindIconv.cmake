include(FindPackageHandleStandardArgs)

file(
	GLOB
	ICONV_INCLUDE_PATHS
	$ENV{ICONVDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/iconv*/include
	$ENV{ProgramFiles}/iconv*/include
	$ENV{ProgramW6432}/libiconv*/include
	$ENV{ProgramFiles}/libiconv*/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
)

file(
	GLOB
	ICONV_LIBRARY_PATHS
	$ENV{ICONVDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/iconv*/lib
	$ENV{ProgramFiles}/iconv*/lib
	$ENV{ProgramW6432}/libiconv*/lib
	$ENV{ProgramFiles}/libiconv*/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_path(
	ICONV_INCLUDE_DIRS
	NAMES
	iconv.h
	HINTS
	${ICONV_INCLUDE_PATHS}
)

find_library(
	ICONV_LIBRARIES
	NAMES
	iconv libiconv libiconv_a
	HINTS
	${ICONV_LIBRARY_PATHS}
)

find_package_handle_standard_args(
	ICONV
	DEFAULT_MSG
	ICONV_INCLUDE_DIRS
	ICONV_LIBRARIES
)

mark_as_advanced(
	ICONV_INCLUDE_DIRS
	ICONV_LIBRARIES
) 
