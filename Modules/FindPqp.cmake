include(FindPackageHandleStandardArgs)

file(
	GLOB
	PQP_INCLUDE_PATHS
	$ENV{PQPDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{SystemDrive}/pqp*/include
	$ENV{ProgramW6432}/pqp*/include
	$ENV{ProgramFiles}/pqp*/include
)

file(
	GLOB
	PQP_LIBRARY_PATHS
	$ENV{PQPDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{SystemDrive}/pqp*/lib
	$ENV{ProgramW6432}/pqp*/lib
	$ENV{ProgramFiles}/pqp*/lib
)

find_path(
	PQP_INCLUDE_DIRS
	NAMES
	PQP.h
	HINTS
	${PQP_INCLUDE_PATHS}
)

find_library(
	PQP_LIBRARIES
	NAMES
	PQP
	HINTS
	${PQP_LIBRARY_PATHS}
)

find_package_handle_standard_args(
	PQP
	DEFAULT_MSG
	PQP_INCLUDE_DIRS
	PQP_LIBRARIES
)

mark_as_advanced(
	PQP_INCLUDE_DIRS
	PQP_LIBRARIES
) 
