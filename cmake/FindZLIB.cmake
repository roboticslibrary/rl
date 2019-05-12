include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ZLIB_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	ZLIB_INCLUDE_HINTS
	$ENV{ZLIB_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ZLIB_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ZLIB_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	ZLIB_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
	${CMAKE_OSX_SYSROOT}/usr/include
)

find_path(
	ZLIB_INCLUDE_DIRS
	NAMES
	zlib.h
	HINTS
	${ZLIB_INCLUDE_HINTS}
	PATHS
	${ZLIB_INCLUDE_PATHS}
)

mark_as_advanced(ZLIB_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ZLIB_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	ZLIB_LIBRARY_HINTS
	$ENV{ZLIB_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ZLIB_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ZLIB_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	ZLIB_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_library(
	ZLIB_LIBRARY_DEBUG
	NAMES
	zd zlibd zlib_ad zlibstaticd zlibwapid zdlld
	HINTS
	${ZLIB_LIBRARY_HINTS}
	PATHS
	${ZLIB_LIBRARY_PATHS}
)

find_library(
	ZLIB_LIBRARY_RELEASE
	NAMES
	z zlib zlib_a zlibstatic zlibwapi zdll
	HINTS
	${ZLIB_LIBRARY_HINTS}
	PATHS
	${ZLIB_LIBRARY_PATHS}
)

select_library_configurations(ZLIB)

find_package_handle_standard_args(
	ZLIB
	FOUND_VAR ZLIB_FOUND
	REQUIRED_VARS ZLIB_INCLUDE_DIRS ZLIB_LIBRARIES
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
	
	set_target_properties(
		ZLIB::ZLIB PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${ZLIB_INCLUDE_DIRS}"
	)
endif()
