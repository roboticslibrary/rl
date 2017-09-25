include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/cifx
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/cifX*/${CMAKE_INSTALL_INCLUDEDIR}/cifx
		${PATH}/cifX*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND cifX_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	cifX_INCLUDE_HINTS
	$ENV{cifX_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/cifx
	$ENV{cifX_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/cifx
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/cifX*/${CMAKE_INSTALL_INCLUDEDIR}/cifx
		${PATH}/cifX*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND cifX_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}/cifx
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND cifX_INCLUDE_HINTS ${HINTS})
endforeach()

find_path(
	cifX_INCLUDE_DIRS
	NAMES
	cifXUser.h
	HINTS
	${cifX_INCLUDE_HINTS}
	PATHS
	$ENV{HOME}/include/cifx
	/usr/local/include/cifx
	/opt/local/include/cifx
	/usr/include/cifx
	"$ENV{ProgramW6432}/cifX Device Driver/SDK/includes"
	"$ENV{ProgramFiles}/cifX Device Driver/SDK/includes"
)

mark_as_advanced(cifX_INCLUDE_DIRS)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(cifX_ARCHITECTURE "x64")
else()
	set(cifX_ARCHITECTURE "x86")
endif()

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libs
		${PATH}/libs/${cifX_ARCHITECTURE}
		${PATH}/cifX*/${CMAKE_INSTALL_LIBDIR}
		${PATH}/cifX*/libs
		${PATH}/cifX*/libs/${cifX_ARCHITECTURE}
	)
	list(APPEND cifX_LIBRARY_HINTS ${HINTS})
endforeach()

list(
	APPEND
	cifX_LIBRARY_HINTS
	$ENV{cifX_DIR}/${CMAKE_INSTALL_LIBDIR}
	$ENV{cifX_DIR}/libs
	$ENV{cifX_DIR}/libs/${cifX_ARCHITECTURE}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_LIBDIR}
		${PATH}/libs
		${PATH}/libs/${cifX_ARCHITECTURE}
		${PATH}/cifX*/${CMAKE_INSTALL_LIBDIR}
		${PATH}/cifX*/libs
		${PATH}/cifX*/libs/${cifX_ARCHITECTURE}
	)
	list(APPEND cifX_LIBRARY_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_LIBDIR}
		${PATH}/../libs
		${PATH}/../libs/${cifX_ARCHITECTURE}
	)
	list(APPEND cifX_LIBRARY_HINTS ${HINTS})
endforeach()

find_library(
	cifX_LIBRARY_RELEASE
	NAMES
	cifx cifx32dll
	HINTS
	${cifX_LIBRARY_HINTS}
	PATHS
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	"$ENV{ProgramW6432}/cifX Device Driver/SDK/libs/${cifX_ARCHITECTURE}"
	"$ENV{ProgramFiles}/cifX Device Driver/SDK/libs/${cifX_ARCHITECTURE}"
)

select_library_configurations(cifX)

find_package_handle_standard_args(
	cifX
	FOUND_VAR cifX_FOUND
	REQUIRED_VARS cifX_INCLUDE_DIRS cifX_LIBRARIES
)

if(cifX_FOUND AND NOT TARGET cifX::cifX)
	add_library(cifX::cifX UNKNOWN IMPORTED)
	
	if(cifX_LIBRARY_RELEASE)
		set_property(TARGET cifX::cifX APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(cifX::cifX PROPERTIES IMPORTED_LOCATION_RELEASE "${cifX_LIBRARY_RELEASE}")
	endif()
	
	set_target_properties(
		cifX::cifX PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${cifX_INCLUDE_DIRS}"
	)
endif()
