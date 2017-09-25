include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/NLopt*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND NLopt_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	NLopt_INCLUDE_HINTS
	$ENV{NLopt_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/NLopt*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND NLopt_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND NLopt_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	NLOPT_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
)

find_path(
	NLOPT_INCLUDE_DIRS
	NAMES
	nlopt.h
	HINTS
	${NLOPT_INCLUDE_HINTS}
	PATHS
	${NLOPT_INCLUDE_PATHS}
)

mark_as_advanced(NLOPT_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/NLopt*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND NLopt_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	NLopt_LIBRARY_HINTS
	$ENV{NLopt_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/NLopt*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND NLopt_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND NLopt_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	NLOPT_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	NLOPT_LIBRARY_DEBUG
	NAMES
	nloptd nlopt_cxxd
	HINTS
	${NLOPT_LIBRARY_HINTS}
	PATHS
	${NLOPT_LIBRARY_PATHS}
)
find_library(
	NLOPT_LIBRARY_RELEASE
	NAMES
	nlopt nlopt_cxx
	HINTS
	${NLOPT_LIBRARY_HINTS}
	PATHS
	${NLOPT_LIBRARY_PATHS}
)

select_library_configurations(NLOPT)

find_package_handle_standard_args(
	NLopt
	FOUND_VAR NLOPT_FOUND
	REQUIRED_VARS NLOPT_INCLUDE_DIRS NLOPT_LIBRARIES
)

if(NLOPT_FOUND AND NOT TARGET NLopt::nlopt)
	add_library(NLopt::nlopt UNKNOWN IMPORTED)
	
	if(NLOPT_LIBRARY_RELEASE)
		set_property(TARGET NLopt::nlopt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(NLopt::nlopt PROPERTIES IMPORTED_LOCATION_RELEASE "${NLOPT_LIBRARY_RELEASE}")
	endif()
	
	if(NLOPT_LIBRARY_DEBUG)
		set_property(TARGET NLopt::nlopt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(NLopt::nlopt PROPERTIES IMPORTED_LOCATION_DEBUG "${NLOPT_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		NLopt::nlopt PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${NLOPT_INCLUDE_DIRS}"
	)
endif()
