include(CheckCSourceRuns)
include(CheckSymbolExists)
include(CMakePushCheckState)
include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	ODE_INCLUDE_DIRS
	NAMES
	ode/ode.h
)

mark_as_advanced(ODE_INCLUDE_DIRS)

find_library(
	ODE_LIBRARY_DEBUG
	NAMES
	ode_doubled ode_singled oded
)

find_library(
	ODE_LIBRARY_RELEASE
	NAMES
	ode_double ode_single ode
)

select_library_configurations(ODE)

if(ODE_INCLUDE_DIRS AND ODE_LIBRARIES)
	cmake_push_check_state(RESET)
	set(CMAKE_REQUIRED_INCLUDES ${ODE_INCLUDE_DIRS})
	check_symbol_exists(dDOUBLE "ode/precision.h" _ODE_HAVE_dDOUBLE)
	
	if(NOT _ODE_HAVE_dDOUBLE)
		check_symbol_exists(dSINGLE "ode/precision.h" _ODE_HAVE_dSINGLE)
		
		if(NOT _ODE_HAVE_dSINGLE)
			set(CMAKE_REQUIRED_DEFINITIONS -DdDOUBLE)
			set(CMAKE_REQUIRED_LIBRARIES ${ODE_LIBRARIES})
			check_c_source_runs("
				#include <ode/ode.h>
				int main() { return 1 == dCheckConfiguration(\"ODE_double_precision\") ? 0 : 1; }
			" _ODE_DOUBLE_PRECISION)
			
			if(_ODE_DOUBLE_PRECISION)
				set(ODE_DEFINITIONS -DdDOUBLE)
			else()
				set(CMAKE_REQUIRED_DEFINITIONS -DdSINGLE)
				check_c_source_runs("
					#include <ode/ode.h>
					int main() { return 1 == dCheckConfiguration(\"ODE_single_precision\") ? 0 : 1; }
				" _ODE_SINGLE_PRECISION)
				
				if(_ODE_SINGLE_PRECISION)
					set(ODE_DEFINITIONS -DdSINGLE)
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

mark_as_advanced(ODE_DEFINITIONS)

if(ODE_INCLUDE_DIRS AND EXISTS "${ODE_INCLUDE_DIRS}/ode/version.h")
	file(STRINGS "${ODE_INCLUDE_DIRS}/ode/version.h" _ODE_VERSION_DEFINE REGEX "#define[\t ]+dODE_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "#define[\t ]+dODE_VERSION[\t ]+\"([^\"]*)\".*" "\\1" ODE_VERSION "${_ODE_VERSION_DEFINE}")
	unset(_ODE_VERSION_DEFINE)
endif()

find_package_handle_standard_args(
	ODE
	FOUND_VAR ODE_FOUND
	REQUIRED_VARS ODE_INCLUDE_DIRS ODE_LIBRARIES
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
	
	set_target_properties(
		ODE::ODE PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${ODE_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
	)
endif()
