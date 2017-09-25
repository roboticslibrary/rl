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
	list(APPEND zlib_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	zlib_INCLUDE_HINTS
	$ENV{zlib_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND zlib_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND zlib_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	zlib_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
	$ENV{ProgramW6432}/GnuWin32/include
	$ENV{ProgramFiles}/GnuWin32/include
)

find_path(
	zlib_INCLUDE_DIRS
	NAMES
	zlib.h
	HINTS
	${zlib_INCLUDE_HINTS}
	PATHS
	${zlib_INCLUDE_PATHS}
)

mark_as_advanced(zlib_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND zlib_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	zlib_LIBRARY_HINTS
	$ENV{zlib_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/zlib*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND zlib_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND zlib_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	zlib_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	$ENV{ProgramW6432}/GnuWin32/lib
	$ENV{ProgramFiles}/GnuWin32/lib
)

find_library(
	zlib_LIBRARY_DEBUG
	NAMES
	zd zlibd zlib_ad zlibstaticd zlibwapid zdlld
	HINTS
	${zlib_LIBRARY_HINTS}
	PATHS
	${zlib_LIBRARY_PATHS}
)

find_library(
	zlib_LIBRARY_RELEASE
	NAMES
	z zlib zlib_a zlibstatic zlibwapi zdll
	HINTS
	${zlib_LIBRARY_HINTS}
	PATHS
	${zlib_LIBRARY_PATHS}
)

select_library_configurations(zlib)

find_package_handle_standard_args(
	zlib
	FOUND_VAR zlib_FOUND
	REQUIRED_VARS zlib_INCLUDE_DIRS zlib_LIBRARIES
)

if(zlib_FOUND AND NOT TARGET zlib::zlib)
	add_library(zlib::zlib UNKNOWN IMPORTED)
	
	if(zlib_LIBRARY_RELEASE)
		set_property(TARGET zlib::zlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(zlib::zlib PROPERTIES IMPORTED_LOCATION_RELEASE "${zlib_LIBRARY_RELEASE}")
	endif()
	
	if(zlib_LIBRARY_DEBUG)
		set_property(TARGET zlib::zlib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(zlib::zlib PROPERTIES IMPORTED_LOCATION_DEBUG "${zlib_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		zlib::zlib PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${zlib_INCLUDE_DIRS}"
	)
endif()
