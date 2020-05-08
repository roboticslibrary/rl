include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/SoQt*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND SoQt_INCLUDE_HINTS ${HINTS})
endforeach()

list(APPEND SoQt_INCLUDE_HINTS $ENV{SoQt_DIR}/${CMAKE_INSTALL_INCLUDEDIR})

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/SoQt*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND SoQt_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND SoQt_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	SoQt_INCLUDE_PATHS
	$ENV{COIN3DDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/opt/local/include
	/usr/include/Coin*
	/usr/include
)

find_path(
	SoQt_INCLUDE_DIRS
	NAMES
	Inventor/Qt/SoQt.h
	HINTS
	${SoQt_INCLUDE_HINTS}
	PATHS
	${SoQt_INCLUDE_PATHS}
)

mark_as_advanced(SoQt_INCLUDE_DIRS)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/SoQt*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND SoQt_LIBRARY_HINTS ${HINTS})
endforeach()

list(APPEND SoQt_LIBRARY_HINTS $ENV{SoQt_DIR}/${CMAKE_INSTALL_LIBDIR})

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/SoQt*/${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND SoQt_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
	)
	list(APPEND SoQt_LIBRARY_HINTS ${HINTS})
endforeach()

file(
	GLOB
	SoQt_LIBRARY_PATHS
	$ENV{COIN3DDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
)

find_library(
	SoQt_LIBRARY_DEBUG
	NAMES
	SoQtd SoQt1d
	HINTS
	${SoQt_LIBRARY_HINTS}
	PATHS
	${SoQt_LIBRARY_PATHS}
)

find_library(
	SoQt_LIBRARY_RELEASE
	NAMES
	SoQt SoQt1
	HINTS
	HINTS
	${SoQt_LIBRARY_HINTS}
	PATHS
	${SoQt_LIBRARY_PATHS}
)

select_library_configurations(SoQt)

set(SoQt_DEFINITIONS -DSOQT_DLL)

mark_as_advanced(SoQt_DEFINITIONS) 

if(EXISTS "${SoQt_INCLUDE_DIRS}/Inventor/Qt/SoQtBasic.h")
	file(STRINGS "${SoQt_INCLUDE_DIRS}/Inventor/Qt/SoQtBasic.h" GUI_TOOLKIT_VERSION_DEFINE REGEX "#define GUI_TOOLKIT_VERSION.*[0-9]+.*")
	string(REGEX REPLACE ".*([0-9])([0-9])([0-9]).*" "\\1" GUI_TOOLKIT_VERSION_MAJOR "${GUI_TOOLKIT_VERSION_DEFINE}")
	set(SoQt${GUI_TOOLKIT_VERSION_MAJOR}_FOUND ON)
endif()

find_package_handle_standard_args(
	SoQt
	FOUND_VAR SoQt_FOUND
	REQUIRED_VARS SoQt_INCLUDE_DIRS SoQt_LIBRARIES
)

if(SoQt_FOUND AND NOT TARGET SoQt::SoQt)
	add_library(SoQt::SoQt UNKNOWN IMPORTED)
	
	if(SoQt_LIBRARY_RELEASE)
		set_property(TARGET SoQt::SoQt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(SoQt::SoQt PROPERTIES IMPORTED_LOCATION_RELEASE "${SoQt_LIBRARY_RELEASE}")
	endif()
	
	if(SoQt_LIBRARY_DEBUG)
		set_property(TARGET SoQt::SoQt APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(SoQt::SoQt PROPERTIES IMPORTED_LOCATION_DEBUG "${SoQt_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		SoQt::SoQt PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${SoQt_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${SoQt_INCLUDE_DIRS}"
	)
endif()
