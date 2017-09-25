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
	list(APPEND libiconv_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	libiconv_INCLUDE_HINTS
	$ENV{libiconv_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND libiconv_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND libiconv_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	libiconv_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
)

find_path(
	libiconv_INCLUDE_DIRS
	NAMES
	iconv.h
	HINTS
	${libiconv_INCLUDE_HINTS}
	PATHS
	${libiconv_INCLUDE_PATHS}
)

mark_as_advanced(libiconv_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libiconv_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	libiconv_LIBRARY_HINTS
	$ENV{libiconv_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libiconv*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libiconv_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libiconv_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	libiconv_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_library(
	libiconv_LIBRARY_DEBUG
	NAMES
	iconvd libiconvd libiconvd_a
	HINTS
	${libiconv_LIBRARY_HINTS}
	PATHS
	${libiconv_LIBRARY_PATHS}
)

find_library(
	libiconv_LIBRARY_RELEASE
	NAMES
	iconv libiconv liblibiconv_a
	HINTS
	${libiconv_LIBRARY_HINTS}
	PATHS
	${libiconv_LIBRARY_PATHS}
)

select_library_configurations(libiconv)

find_package_handle_standard_args(
	libiconv
	FOUND_VAR libiconv_FOUND
	REQUIRED_VARS libiconv_INCLUDE_DIRS libiconv_LIBRARIES
)

if(libiconv_FOUND AND NOT TARGET libiconv::libiconv)
	add_library(libiconv::libiconv UNKNOWN IMPORTED)
	
	if(libiconv_LIBRARY_RELEASE)
		set_property(TARGET libiconv::libiconv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(libiconv::libiconv PROPERTIES IMPORTED_LOCATION_RELEASE "${libiconv_LIBRARY_RELEASE}")
	endif()
	
	if(libiconv_LIBRARY_DEBUG)
		set_property(TARGET libiconv::libiconv APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(libiconv::libiconv PROPERTIES IMPORTED_LOCATION_DEBUG "${libiconv_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		libiconv::libiconv PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${libiconv_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${libiconv_INCLUDE_DIRS}"
	)
endif()
