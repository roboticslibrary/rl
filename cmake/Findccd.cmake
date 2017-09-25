include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/ccd*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND CCD_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	CCD_INCLUDE_HINTS
	$ENV{ccd_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/ccd*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND CCD_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND CCD_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	CCD_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
)

find_path(
	CCD_INCLUDE_DIRS
	NAMES
	ccd/ccd.h
	HINTS
	${CCD_INCLUDE_HINTS}
	PATHS
	${CCD_INCLUDE_PATHS}
)

mark_as_advanced(CCD_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/ccd*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND CCD_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	CCD_LIBRARY_HINTS
	$ENV{ccd_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/ccd*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND CCD_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND CCD_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	CCD_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	CCD_LIBRARY_DEBUG
	NAMES
	ccdd
	HINTS
	${CCD_LIBRARY_HINTS}
	PATHS
	${CCD_LIBRARY_PATHS}
)

find_library(
	CCD_LIBRARY_RELEASE
	NAMES
	ccd
	HINTS
	${CCD_LIBRARY_HINTS}
	PATHS
	${CCD_LIBRARY_PATHS}
)

select_library_configurations(CCD)

find_package_handle_standard_args(
	ccd
	FOUND_VAR ccd_FOUND
	REQUIRED_VARS CCD_INCLUDE_DIRS CCD_LIBRARIES
)

if(ccd_FOUND AND NOT TARGET ccd::ccd)
	add_library(ccd::ccd UNKNOWN IMPORTED)
	
	if(CCD_LIBRARY_RELEASE)
		set_property(TARGET ccd::ccd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_RELEASE "${CCD_LIBRARY_RELEASE}")
	endif()
	
	if(CCD_LIBRARY_DEBUG)
		set_property(TARGET ccd::ccd APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(ccd::ccd PROPERTIES IMPORTED_LOCATION_DEBUG "${CCD_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		ccd::ccd PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${CCD_INCLUDE_DIRS}"
	)
endif()
