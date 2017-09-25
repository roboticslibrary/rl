include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/RTAI*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND RTAI_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	RTAI_INCLUDE_HINTS
	$ENV{RTAI_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/RTAI*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND RTAI_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND RTAI_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	RTAI_INCLUDE_PATHS
	$ENV{RTAI_DIR}/include
	$ENV{RTAIDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/realtime/include
	/usr/include/rtai
	/usr/include
)

find_path(
	RTAI_INCLUDE_DIRS
	NAMES
	rtai_lxrt.h
	HINTS
	${RTAI_INCLUDE_HINTS}
	PATHS
	${RTAI_INCLUDE_PATHS}
)

mark_as_advanced(RTAI_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/RTAI*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND RTAI_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	RTAI_LIBRARY_HINTS
	$ENV{RTAI_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/RTAI*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND RTAI_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND RTAI_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	RTAI_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/realtime/lib
	/usr/lib
)

find_library(
	RTAI_LIBRARY_DEBUG
	NAMES
	lxrtd
	HINTS
	${RTAI_LIBRARY_HINTS}
	PATHS
	${RTAI_LIBRARY_PATHS}
)

find_library(
	RTAI_LIBRARY_RELEASE
	NAMES
	lxrt
	HINTS
	${RTAI_LIBRARY_HINTS}
	PATHS
	${RTAI_LIBRARY_PATHS}
)

select_library_configurations(RTAI)

set(RTAI_LIBRARIES ${RTAI_LIBRARIES} rt)

find_package_handle_standard_args(
	RTAI
	FOUND_VAR RTAI_FOUND
	REQUIRED_VARS RTAI_INCLUDE_DIRS RTAI_LIBRARIES
)

if(RTAI_FOUND AND NOT TARGET RTAI::lxrt)
	add_library(RTAI::lxrt UNKNOWN IMPORTED)
	
	if(RTAI_LIBRARY_RELEASE)
		set_property(TARGET RTAI::lxrt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(RTAI::lxrt PROPERTIES IMPORTED_LOCATION_RELEASE "${RTAI_LIBRARY_RELEASE}")
	endif()
	
	if(RTAI_LIBRARY_DEBUG)
		set_property(TARGET RTAI::lxrt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(RTAI::lxrt PROPERTIES IMPORTED_LOCATION_DEBUG "${RTAI_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		RTAI::lxrt PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${RTAI_INCLUDE_DIRS}"
	)
endif()
