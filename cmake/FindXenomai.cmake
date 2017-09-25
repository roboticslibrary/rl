include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Xenomai*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Xenomai_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Xenomai_INCLUDE_HINTS
	$ENV{Xenomai_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Xenomai*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Xenomai_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Xenomai_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Xenomai_INCLUDE_PATHS
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/xenomai/include
	/usr/include/xenomai
	/usr/include
)

find_path(
	Xenomai_INCLUDE_DIRS
	NAMES
	native/task.h
	HINTS
	${Xenomai_INCLUDE_HINTS}
	PATHS
	${Xenomai_INCLUDE_PATHS}
)

mark_as_advanced(Xenomai_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Xenomai*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Xenomai_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Xenomai_LIBRARY_HINTS
	$ENV{Xenomai_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Xenomai*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Xenomai_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Xenomai_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Xenomai_LIBRARY_PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/xenomai/lib
	/usr/lib
)

find_library(
	Xenomai_NATIVE_LIBRARY_DEBUG
	NAMES
	natived
	HINTS
	${Xenomai_LIBRARY_HINTS}
	PATHS
	${Xenomai_LIBRARY_PATHS}
)

find_library(
	Xenomai_NATIVE_LIBRARY_RELEASE
	NAMES
	native
	HINTS
	${Xenomai_LIBRARY_HINTS}
	PATHS
	${Xenomai_LIBRARY_PATHS}
)

select_library_configurations(Xenomai_NATIVE)

find_library(
	Xenomai_XENOMAI_LIBRARY_DEBUG
	NAMES
	xenomaid
	HINTS
	${Xenomai_LIBRARY_HINTS}
	PATHS
	${Xenomai_LIBRARY_PATHS}
)

find_library(
	Xenomai_XENOMAI_LIBRARY_RELEASE
	NAMES
	xenomai
	HINTS
	${Xenomai_LIBRARY_HINTS}
	PATHS
	${Xenomai_LIBRARY_PATHS}
)

select_library_configurations(Xenomai_XENOMAI)

set(Xenomai_LIBRARIES ${Xenomai_NATIVE_LIBRARIES} ${Xenomai_XENOMAI_LIBRARIES} pthread rt)

set(Xenomai_DEFINITIONS -D__XENO__ -D_GNU_SOURCE -D_REENTRANT)

mark_as_advanced(Xenomai_DEFINITIONS) 

find_package_handle_standard_args(
	Xenomai
	FOUND_VAR Xenomai_FOUND
	REQUIRED_VARS Xenomai_INCLUDE_DIRS Xenomai_NATIVE_LIBRARY Xenomai_XENOMAI_LIBRARY
)

if(Xenomai_FOUND AND NOT TARGET Xenomai::native)
	add_library(Xenomai::native UNKNOWN IMPORTED)
	
	if(Xenomai_NATIVE_LIBRARY_RELEASE)
		set_property(TARGET Xenomai::native APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Xenomai::native PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_NATIVE_LIBRARY_RELEASE}")
	endif()
	
	if(Xenomai_LIBRARY_DEBUG)
		set_property(TARGET Xenomai::native APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Xenomai::native PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_NATIVE_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Xenomai::native PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Xenomai_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Xenomai_INCLUDE_DIRS}"
	)
endif()

if(Xenomai_FOUND AND NOT TARGET Xenomai::xenomai)
	add_library(Xenomai::xenomai UNKNOWN IMPORTED)
	
	if(Xenomai_LIBRARY_RELEASE)
		set_property(TARGET Xenomai::xenomai APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LOCATION_RELEASE "${Xenomai_XENOMAI_LIBRARY_RELEASE}")
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "Xenomai::native")
	endif()
	
	if(Xenomai_LIBRARY_DEBUG)
		set_property(TARGET Xenomai::xenomai APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LOCATION_DEBUG "${Xenomai_XENOMAI_LIBRARY_DEBUG}")
		set_target_properties(Xenomai::xenomai PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "Xenomai::native")
	endif()
	
	set_target_properties(
		Xenomai::xenomai PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Xenomai_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Xenomai_INCLUDE_DIRS}"
	)
endif()
