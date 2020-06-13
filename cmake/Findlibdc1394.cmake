include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libdc1394*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND libdc1394_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	libdc1394_INCLUDE_HINTS
	$ENV{libdc1394_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libdc1394*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND libdc1394_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND libdc1394_INCLUDE_HINTS ${HINTS})
endforeach()

find_path(
	libdc1394_INCLUDE_DIRS
	dc1394/dc1394.h
	HINTS
	${libdc1394_INCLUDE_HINTS}
	PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
)

mark_as_advanced(libdc1394_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libdc1394*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libdc1394_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	libdc1394_LIBRARY_HINTS
	$ENV{libdc1394_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libdc1394*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libdc1394_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND libdc1394_LIBRARY_HINTS ${HINTS})
endforeach()

find_library(
	libdc1394_LIBRARY_RELEASE
	NAMES
	dc1394
	HINTS
	${libdc1394_LIBRARY_HINTS}
	PATHS
	$ENV{libdc1394_DIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

mark_as_advanced(libdc1394_LIBRARY_RELEASE)

select_library_configurations(libdc1394)

find_package_handle_standard_args(
	libdc1394
	FOUND_VAR libdc1394_FOUND
	REQUIRED_VARS libdc1394_INCLUDE_DIRS libdc1394_LIBRARIES
)

if(libdc1394_FOUND AND NOT TARGET libdc1394::libdc1394)
	add_library(libdc1394::libdc1394 UNKNOWN IMPORTED)
	
	if(libdc1394_LIBRARY_RELEASE)
		set_property(TARGET libdc1394::libdc1394 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(libdc1394::libdc1394 PROPERTIES IMPORTED_LOCATION_RELEASE "${libdc1394_LIBRARY_RELEASE}")
	endif()
	
	set_target_properties(
		libdc1394::libdc1394 PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${libdc1394_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${libdc1394_INCLUDE_DIRS}"
	)
endif()
