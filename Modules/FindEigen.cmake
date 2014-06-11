include(FindPackageHandleStandardArgs)

file(
	GLOB
	EIGEN_INCLUDE_PATHS
	$ENV{EIGENDIR}
	$ENV{HOME}/include
	/usr/local/include/eigen3
	/usr/local/include/eigen*
	/usr/local/include
	/usr/include/eigen3
	/usr/include/eigen*
	/usr/include
	/opt/local/include/eigen*
	$ENV{ProgramW6432}/eigen*/include/eigen3
	$ENV{ProgramFiles}/eigen*/include/eigen3
	$ENV{ProgramW6432}/eigen*
	$ENV{ProgramFiles}/eigen*
)

find_path(
	EIGEN_INCLUDE_DIRS
	NAMES
	Eigen/Core
	HINTS
	${EIGEN_INCLUDE_PATHS}
)

find_package_handle_standard_args(
	Eigen
	DEFAULT_MSG
	EIGEN_INCLUDE_DIRS
)

mark_as_advanced(
	EIGEN_FOUND
	EIGEN_INCLUDE_DIRS
) 
