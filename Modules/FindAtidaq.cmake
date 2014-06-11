include(FindPackageHandleStandardArgs)

find_path(
	ATIDAQ_INCLUDE_DIRS
	NAMES
	atidaq/ftconfig.h
	HINTS
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
)

find_library(
	ATIDAQ_LIBRARIES
	NAMES
	atidaq
	HINTS
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
)

find_package_handle_standard_args(
	ATIDAQ
	DEFAULT_MSG
	ATIDAQ_INCLUDE_DIRS
	ATIDAQ_LIBRARIES
)

mark_as_advanced(
	ATIDAQ_INCLUDE_DIRS
	ATIDAQ_LIBRARIES
) 
