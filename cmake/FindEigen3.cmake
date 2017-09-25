include(FindPackageHandleStandardArgs)
include(GNUInstallDirs)
include(SelectLibraryConfigurations)

foreach(PATH ${CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/eigen3
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Eigen*/${CMAKE_INSTALL_INCLUDEDIR}/eigen3
		${PATH}/Eigen*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND EIGEN3_INCLUDE_HINTS ${HINTS})
endforeach()

list(
	APPEND
	EIGEN3_INCLUDE_HINTS
	$ENV{Eigen3_DIR}/${CMAKE_INSTALL_INCLUDEDIR}/eigen3
	$ENV{Eigen3_DIR}/${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(PATH $ENV{CMAKE_PREFIX_PATH})
	file(
		GLOB
		HINTS
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}/eigen3
		${PATH}/${CMAKE_INSTALL_INCLUDEDIR}
		${PATH}/Eigen*/${CMAKE_INSTALL_INCLUDEDIR}/eigen3
		${PATH}/Eigen*/${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND EIGEN3_INCLUDE_HINTS ${HINTS})
endforeach()

foreach(PATH $ENV{PATH})
	file(
		GLOB
		HINTS
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}/eigen3
		${PATH}/../${CMAKE_INSTALL_INCLUDEDIR}
	)
	list(APPEND EIGEN3_INCLUDE_HINTS ${HINTS})
endforeach()

file(
	GLOB
	EIGEN3_INCLUDE_PATHS
	$ENV{EIGEN3_ROOT}/include/eigen3
	$ENV{EIGEN3_ROOT}/include
	$ENV{EIGEN3_ROOT}
	$ENV{EIGEN3_ROOT_DIR}/include/eigen3
	$ENV{EIGEN3_ROOT_DIR}/include
	$ENV{EIGEN3_ROOT_DIR}
	$ENV{HOME}/include
	/usr/local/include/eigen3
	/usr/local/include/eigen*
	/usr/local/include
	/opt/local/include/eigen3
	/opt/local/include/eigen*
	/opt/local/include
	/usr/include/eigen3
	/usr/include/eigen*
	/usr/include
)

find_path(
	EIGEN3_INCLUDE_DIRS
	NAMES
	Eigen/Core
	HINTS
	${EIGEN3_INCLUDE_HINTS}
	PATHS
	${EIGEN3_INCLUDE_PATHS}
)

mark_as_advanced(EIGEN3_INCLUDE_DIRS)

find_package_handle_standard_args(
	Eigen3
	FOUND_VAR EIGEN3_FOUND
	REQUIRED_VARS EIGEN3_INCLUDE_DIRS
)

if(EIGEN3_FOUND AND NOT TARGET Eigen3::Eigen)
	add_library(Eigen3::Eigen UNKNOWN IMPORTED)
	set_target_properties(
		Eigen3::Eigen
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
	)
endif()
