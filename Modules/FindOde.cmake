include(FindPackageHandleStandardArgs)

file(
	GLOB
	ODE_INCLUDE_PATHS
	$ENV{ODEDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{SystemDrive}/ode*/include
	$ENV{ProgramW6432}/ode*/include
	$ENV{ProgramFiles}/ode*/include
)

file(
	GLOB
	ODE_LIBRARY_PATHS
	$ENV{ODEDIR}/lib
	$ENV{HOME}/lib/ReleaseSingleDLL
	$ENV{HOME}/lib/DebugSingleDLL
	$ENV{HOME}/lib/ReleaseDoubleDLL	
	$ENV{HOME}/lib/DebugDoubleDLL
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{SystemDrive}/ode*/lib/ReleaseSingleDLL
	$ENV{ProgramW6432}/ode*/lib/ReleaseSingleDLL
	$ENV{ProgramFiles}/ode*/lib/ReleaseSingleDLL
	$ENV{SystemDrive}/ode*/lib/DebugSingleDLL
	$ENV{ProgramW6432}/ode*/lib/DebugSingleDLL
	$ENV{ProgramFiles}/ode*/lib/DebugSingleDLL
	$ENV{SystemDrive}/ode*/lib/ReleaseDoubleDLL
	$ENV{ProgramW6432}/ode*/lib/ReleaseDoubleDLL
	$ENV{ProgramFiles}/ode*/lib/ReleaseDoubleDLL
	$ENV{SystemDrive}/ode*/lib/DebugDoubleDLL
	$ENV{ProgramW6432}/ode*/lib/DebugDoubleDLL
	$ENV{ProgramFiles}/ode*/lib/DebugDoubleDLL
	$ENV{SystemDrive}/ode*/lib
	$ENV{ProgramW6432}/ode*/lib
	$ENV{ProgramFiles}/ode*/lib
)

find_path(
	ODE_INCLUDE_DIRS
	NAMES
	ode/ode.h
	HINTS
	${ODE_INCLUDE_PATHS}
)

find_library(
	ODE_LIBRARY_DEBUG
	NAMES
	oded ode_singled ode_doubled
	HINTS
	${ODE_LIBRARY_PATHS}
)

find_library(
	ODE_LIBRARY_RELEASE
	NAMES
	ode ode_single ode_double
	HINTS
	${ODE_LIBRARY_PATHS}
)

option(ODE_USE_DOUBLE_PRECISION "ODE use double precision" FALSE)

if(ODE_USE_DOUBLE_PRECISION)
	set(ODE_DEFINITIONS -DdDOUBLE)
else(ODE_USE_DOUBLE_PRECISION)
	set(ODE_DEFINITIONS -DdSINGLE)
endif(ODE_USE_DOUBLE_PRECISION)

if(ODE_LIBRARY_DEBUG AND NOT ODE_LIBRARY_RELEASE)
	set(ODE_LIBRARIES ${ODE_LIBRARY_DEBUG})
endif(ODE_LIBRARY_DEBUG AND NOT ODE_LIBRARY_RELEASE)

if(ODE_LIBRARY_RELEASE AND NOT ODE_LIBRARY_DEBUG)
	set(ODE_LIBRARIES ${ODE_LIBRARY_RELEASE})
endif(ODE_LIBRARY_RELEASE AND NOT ODE_LIBRARY_DEBUG)

if(ODE_LIBRARY_DEBUG AND ODE_LIBRARY_RELEASE)
	set(ODE_LIBRARIES debug ${ODE_LIBRARY_DEBUG} optimized ${ODE_LIBRARY_RELEASE})
endif(ODE_LIBRARY_DEBUG AND ODE_LIBRARY_RELEASE)

find_package_handle_standard_args(
	ODE
	DEFAULT_MSG
	ODE_INCLUDE_DIRS
	ODE_LIBRARIES
)

mark_as_advanced(
	ODE_DEFINITIONS
	ODE_INCLUDE_DIRS
	ODE_LIBRARIES
	ODE_LIBRARY_DEBUG
	ODE_LIBRARY_RELEASE
) 
