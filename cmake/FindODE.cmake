include(CheckCSourceRuns)
include(CheckSymbolExists)
include(CMakePushCheckState)
include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	ODE_INCLUDE_DIR
	NAMES ode/ode.h
)
find_library(
	ODE_LIBRARY_DEBUG
	NAMES ode_doubled ode_singled oded ode_doublesd ode_singlesd odesd
)
find_library(
	ODE_LIBRARY_RELEASE
	NAMES ode_double ode_single ode ode_doubles ode_singles odes
)
select_library_configurations(ODE)

if(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	cmake_push_check_state(RESET)
	set(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIR})
	check_symbol_exists(dDOUBLE "ode/precision.h" _ODE_HAVE_dDOUBLE)
	if(NOT _ODE_HAVE_dDOUBLE)
		check_symbol_exists(dSINGLE "ode/precision.h" _ODE_HAVE_dSINGLE)
		if(NOT _ODE_HAVE_dSINGLE)
			set(CMAKE_REQUIRED_DEFINITIONS -DdDOUBLE)
			set(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARY})
			check_c_source_runs("
				#include <ode/ode.h>
				int main() { return 1 == dCheckConfiguration(\"ODE_double_precision\") ? 0 : 1; }
			" _ODE_DOUBLE_PRECISION)
			if(_ODE_DOUBLE_PRECISION)
				set(ODE_DEFINITIONS -DdDOUBLE)
				set(_ODE_INTERFACE_COMPILE_DEFINITIONS "dDOUBLE")
			else()
				set(CMAKE_REQUIRED_DEFINITIONS -DdSINGLE)
				check_c_source_runs("
					#include <ode/ode.h>
					int main() { return 1 == dCheckConfiguration(\"ODE_single_precision\") ? 0 : 1; }
				" _ODE_SINGLE_PRECISION)
				if(_ODE_SINGLE_PRECISION)
					set(ODE_DEFINITIONS -DdSINGLE)
					set(_ODE_INTERFACE_COMPILE_DEFINITIONS "dSINGLE")
				endif()
				unset(_ODE_SINGLE_PRECISION)
			endif()
			unset(_ODE_DOUBLE_PRECISION)
		endif()
		unset(_ODE_HAVE_dSINGLE)
	endif()
	unset(_ODE_HAVE_dDOUBLE)
	cmake_pop_check_state()
endif()

if(ODE_INCLUDE_DIR AND EXISTS "${ODE_INCLUDE_DIR}/ode/version.h")
	file(STRINGS "${ODE_INCLUDE_DIR}/ode/version.h" _ODE_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+dODE_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+dODE_VERSION[\t ]+\"([^\"]*)\".*" "\\1" ODE_VERSION "${_ODE_VERSION_DEFINE}")
	unset(_ODE_VERSION_DEFINE)
endif()

set(ODE_INCLUDE_DIRS ${ODE_INCLUDE_DIR})
unset(_ODE_INTERFACE_LINK_LIBRARIES)
set(ODE_LIBRARIES ${ODE_LIBRARY})

find_package(ccd QUIET)
if(ccd_FOUND)
	list(APPEND _ODE_INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:ccd::ccd>")
	list(APPEND ODE_LIBRARIES ${CCD_LIBRARIES})
endif()

find_package(Threads QUIET)
if(Threads_FOUND)
	list(APPEND _ODE_INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:Threads::Threads>")
	list(APPEND ODE_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
endif()

find_package_handle_standard_args(
	ODE
	FOUND_VAR ODE_FOUND
	REQUIRED_VARS ODE_INCLUDE_DIR ODE_LIBRARY
	VERSION_VAR ODE_VERSION
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
	set_target_properties(ODE::ODE PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${_ODE_INTERFACE_COMPILE_DEFINITIONS}")
	set_target_properties(ODE::ODE PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}")
	set_target_properties(ODE::ODE PROPERTIES INTERFACE_LINK_LIBRARIES "${_ODE_INTERFACE_LINK_LIBRARIES}")
endif()

mark_as_advanced(ODE_DEFINITIONS)
mark_as_advanced(ODE_INCLUDE_DIR)
unset(_ODE_INTERFACE_COMPILE_DEFINITIONS)
unset(_ODE_INTERFACE_LINK_LIBRARIES)
