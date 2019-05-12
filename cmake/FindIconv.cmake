include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Iconv_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Iconv_INCLUDE_HINTS
	$ENV{Iconv_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Iconv_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Iconv_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Iconv_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
	${CMAKE_OSX_SYSROOT}/usr/include
)

find_path(
	Iconv_INCLUDE_DIRS
	NAMES
	iconv.h
	HINTS
	${Iconv_INCLUDE_HINTS}
	PATHS
	${Iconv_INCLUDE_PATHS}
)

mark_as_advanced(Iconv_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Iconv_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Iconv_LIBRARY_HINTS
	$ENV{Iconv_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Iconv_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Iconv_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Iconv_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_library(
	Iconv_LIBRARY_DEBUG
	NAMES
	iconvd libiconvd libiconvd_a
	HINTS
	${Iconv_LIBRARY_HINTS}
	PATHS
	${Iconv_LIBRARY_PATHS}
)

find_library(
	Iconv_LIBRARY_RELEASE
	NAMES
	iconv libiconv liblibiconv_a
	HINTS
	${Iconv_LIBRARY_HINTS}
	PATHS
	${Iconv_LIBRARY_PATHS}
)

select_library_configurations(Iconv)

find_package_handle_standard_args(
	Iconv
	FOUND_VAR Iconv_FOUND
	REQUIRED_VARS Iconv_INCLUDE_DIRS Iconv_LIBRARIES
)

if(Iconv_FOUND AND NOT TARGET Iconv::Iconv)
	add_library(Iconv::Iconv UNKNOWN IMPORTED)
	
	if(Iconv_LIBRARY_RELEASE)
		set_property(TARGET Iconv::Iconv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Iconv::Iconv PROPERTIES IMPORTED_LOCATION_RELEASE "${Iconv_LIBRARY_RELEASE}")
	endif()
	
	if(Iconv_LIBRARY_DEBUG)
		set_property(TARGET Iconv::Iconv APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Iconv::Iconv PROPERTIES IMPORTED_LOCATION_DEBUG "${Iconv_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Iconv::Iconv PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Iconv_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Iconv_INCLUDE_DIRS}"
	)
endif()
