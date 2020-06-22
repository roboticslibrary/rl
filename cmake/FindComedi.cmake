include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	Comedi_INCLUDE_DIRS
	NAMES
	comedilib.h
)

mark_as_advanced(Comedi_INCLUDE_DIRS)

find_library(
	Comedi_LIBRARY_DEBUG
	NAMES
	comedid
)

find_library(
	Comedi_LIBRARY_RELEASE
	NAMES
	comedi
)

select_library_configurations(Comedi)

find_package_handle_standard_args(
	Comedi
	FOUND_VAR Comedi_FOUND
	REQUIRED_VARS Comedi_INCLUDE_DIRS Comedi_LIBRARIES
)

if(Comedi_FOUND AND NOT TARGET Comedi::Comedi)
	add_library(Comedi::Comedi UNKNOWN IMPORTED)
	
	if(Comedi_LIBRARY_RELEASE)
		set_property(TARGET Comedi::Comedi APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Comedi::Comedi PROPERTIES IMPORTED_LOCATION_RELEASE "${Comedi_LIBRARY_RELEASE}")
	endif()
	
	if(Comedi_LIBRARY_DEBUG)
		set_property(TARGET Comedi::Comedi APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Comedi::Comedi PROPERTIES IMPORTED_LOCATION_DEBUG "${Comedi_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Comedi::Comedi PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Comedi_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Comedi_INCLUDE_DIRS}"
	)
endif()
