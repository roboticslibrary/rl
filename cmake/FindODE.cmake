include(CheckCSourceRuns)
include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/ODE*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ODE_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	ODE_INCLUDE_HINTS
	$ENV{ODE_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/ODE*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ODE_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND ODE_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	ODE_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
)

find_path(
	ODE_INCLUDE_DIRS
	NAMES
	ode/ode.h
	HINTS
	${ODE_INCLUDE_HINTS}
	PATHS
	${ODE_INCLUDE_PATHS}
)

mark_as_advanced(ODE_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/ODE*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ODE_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	ODE_LIBRARY_HINTS
	$ENV{ODE_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/ODE*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ODE_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND ODE_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	ODE_LIBRARY_PATHS
	$ENV{ODE_LIBRARYDIR}
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	ODE_LIBRARY_DEBUG
	NAMES
	ode_doubled ode_singled oded
	HINTS
	${ODE_LIBRARY_HINTS}
	PATHS
	${ODE_LIBRARY_PATHS}
)

find_library(
	ODE_LIBRARY_RELEASE
	NAMES
	ode_double ode_single ode
	HINTS
	${ODE_LIBRARY_HINTS}
	PATHS
	${ODE_LIBRARY_PATHS}
)

select_library_configurations(ODE)

set(CMAKE_REQUIRED_DEFINITIONS -DdDOUBLE)
set(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARIES})

check_c_source_runs("
	#include <ode/ode.h>
	int main() { return 1 == dCheckConfiguration(\"ODE_double_precision\") ? 0 : 1; }
" ODE_DOUBLE_PRECISION)

if(ODE_DOUBLE_PRECISION)
	set(ODE_DEFINITIONS -DdDOUBLE)
else()
	set(ODE_DEFINITIONS -DdSINGLE)
endif()

mark_as_advanced(ODE_DEFINITIONS) 

find_package_handle_standard_args(
	ODE
	FOUND_VAR ODE_FOUND
	REQUIRED_VARS ODE_INCLUDE_DIRS ODE_LIBRARIES
)

if(ODE_FOUND AND NOT TARGET ODE::ODE)
	add_library(ODE::ODE UNKNOWN IMPORTED)
	
	if(ODE_LIBRARY_RELEASE)
		set_property(TARGET ODE::ODE APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(ODE::ODE PROPERTIES IMPORTED_LOCATION_RELEASE "${ODE_LIBRARY_RELEASE}")
	endif()
	
	if(ODE_LIBRARY_DEBUG)
		set_property(TARGET ODE::ODE APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(ODE::ODE PROPERTIES IMPORTED_LOCATION_DEBUG "${ODE_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		ODE::ODE PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${ODE_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
	)
endif()
