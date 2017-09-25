include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/libxml2
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libxml2*/${CMAKE_INSTALL_INCLUDEDIR}/libxml2
		${PATH}/libxml2*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND LIBXML2_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	LIBXML2_INCLUDE_HINTS
	$ENV{libxml2_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/libxml2
	$ENV{libxml2_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/libxml2
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/libxml2*/${CMAKE_INSTALL_INCLUDEDIR}/libxml2
		${PATH}/libxml2*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND LIBXML2_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}/libxml2
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND LIBXML2_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	LIBXML2_INCLUDE_PATHS
	$ENV{HOME}/include/libxml2
	$ENV{HOME}/include
	/usr/local/include/libxml2
	/usr/local/include
	/opt/local/include/libxml2
	/opt/local/include
	/usr/include/libxml2
	/usr/include
)

find_path(
	LIBXML2_INCLUDE_DIRS
	NAMES
	libxml/parser.h
	HINTS
	${LIBXML2_INCLUDE_HINTS}
	PATHS
	${LIBXML2_INCLUDE_PATHS}
)

mark_as_advanced(LIBXML2_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libxml2*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND LIBXML2_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	LIBXML2_LIBRARY_HINTS
	$ENV{LIBXML2_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libxml2*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND LIBXML2_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND LIBXML2_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	LIBXML2_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	LIBXML2_LIBRARY_DEBUG
	NAMES
	libxml2d xml2d
	HINTS
	${LIBXML2_LIBRARY_HINTS}
	PATHS
	${LIBXML2_LIBRARY_PATHS}
)
find_library(
	LIBXML2_LIBRARY_RELEASE
	NAMES
	libxml2 xml2
	HINTS
	${LIBXML2_LIBRARY_HINTS}
	PATHS
	${LIBXML2_LIBRARY_PATHS}
)

select_library_configurations(LIBXML2)

find_package_handle_standard_args(
	libxml2
	FOUND_VAR LIBXML2_FOUND
	REQUIRED_VARS LIBXML2_INCLUDE_DIRS LIBXML2_LIBRARIES
)

if(LIBXML2_FOUND AND NOT TARGET libxml2::libxml2)
	add_library(libxml2::libxml2 UNKNOWN IMPORTED)
	
	if(LIBXML2_LIBRARY_RELEASE)
		set_property(TARGET libxml2::libxml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(libxml2::libxml2 PROPERTIES IMPORTED_LOCATION_RELEASE "${LIBXML2_LIBRARY_RELEASE}")
	endif()
	
	if(LIBXML2_LIBRARY_DEBUG)
		set_property(TARGET libxml2::libxml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(libxml2::libxml2 PROPERTIES IMPORTED_LOCATION_DEBUG "${LIBXML2_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		libxml2::libxml2 PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${LIBXML2_INCLUDE_DIRS}"
	)
endif()
