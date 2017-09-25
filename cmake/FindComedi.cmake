include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Comedi*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Comedi_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Comedi_INCLUDE_HINTS
	$ENV{Comedi_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Comedi*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Comedi_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Comedi_INCLUDE_HINTS ${HINTS})
endforeach()

find_path(
	Comedi_INCLUDE_DIRS
	NAMES
	comedilib.h
	HINTS
	${Comedi_INCLUDE_HINTS}
	PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include
)

mark_as_advanced(Comedi_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Comedi*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Comedi_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Comedi_LIBRARY_HINTS
	$ENV{Comedi_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Comedi*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Comedi_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Comedi_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Comedi_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	Comedi_LIBRARY_DEBUG
	NAMES
	comedid
	HINTS
	${Comedi_LIBRARY_HINTS}
	PATHS
	${Comedi_LIBRARY_PATHS}
)

find_library(
	Comedi_LIBRARY_RELEASE
	NAMES
	comedi
	HINTS
	${Comedi_LIBRARY_HINTS}
	PATHS
	${Comedi_LIBRARY_PATHS}
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
