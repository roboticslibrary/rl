include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Coin*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Coin_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Coin_INCLUDE_HINTS
	$ENV{Coin_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Coin*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Coin_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND Coin_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Coin_INCLUDE_PATHS
	$ENV{COIN3DDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include/Coin*
	/usr/include
)

find_path(
	Coin_INCLUDE_DIRS
	NAMES
	Inventor/So.h
	HINTS
	${Coin_INCLUDE_HINTS}
	PATHS
	${Coin_INCLUDE_PATHS}
)

mark_as_advanced(Coin_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Coin*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Coin_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	Coin_LIBRARY_HINTS
	$ENV{Coin_DIR}/${CMAKE_INSTALL_LIBDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/Coin*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Coin_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND Coin_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	Coin_LIBRARY_PATHS
	$ENV{COIN3DDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	Coin_LIBRARY_DEBUG
	NAMES
	Coind coin2d coin3d Coin4d Inventord
	HINTS
	${Coin_LIBRARY_HINTS}
	PATHS
	${Coin_LIBRARY_PATHS}
)

find_library(
	Coin_LIBRARY_RELEASE
	NAMES
	Coin coin2 coin3 Coin4 Inventor
	HINTS
	${Coin_LIBRARY_HINTS}
	PATHS
	${Coin_LIBRARY_PATHS}
)

select_library_configurations(Coin)

set(Coin_DEFINITIONS -DCOIN_DLL)

mark_as_advanced(Coin_DEFINITIONS)

find_package_handle_standard_args(
	Coin
	FOUND_VAR Coin_FOUND
	REQUIRED_VARS Coin_INCLUDE_DIRS Coin_LIBRARIES
)

if(Coin_FOUND AND NOT TARGET Coin::Coin)
	add_library(Coin::Coin UNKNOWN IMPORTED)
	
	if(Coin_LIBRARY_RELEASE)
		set_property(TARGET Coin::Coin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Coin::Coin PROPERTIES IMPORTED_LOCATION_RELEASE "${Coin_LIBRARY_RELEASE}")
	endif()
	
	if(Coin_LIBRARY_DEBUG)
		set_property(TARGET Coin::Coin APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Coin::Coin PROPERTIES IMPORTED_LOCATION_DEBUG "${Coin_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Coin::Coin PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Coin_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Coin_INCLUDE_DIRS}"
	)
endif()
