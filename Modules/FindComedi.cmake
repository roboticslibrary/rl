include(FindPackageHandleStandardArgs)

find_path(
	COMEDI_INCLUDE_DIRS
	NAMES
	comedilib.h
	HINTS
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
)

find_library(
	COMEDI_LIBRARIES
	NAMES
	comedi
	HINTS
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
)

find_package_handle_standard_args(
	Comedi
	DEFAULT_MSG
	COMEDI_INCLUDE_DIRS
	COMEDI_LIBRARIES
)

mark_as_advanced(
	COMEDI_INCLUDE_DIRS
	COMEDI_LIBRARIES
) 
