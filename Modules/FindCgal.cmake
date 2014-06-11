include(FindPackageHandleStandardArgs)

file(
	GLOB
	CGAL_COMPILER_INCLUDE_PATHS
	$ENV{CGALROOT}/include/CGAL/config/*
	$ENV{HOME}/include/CGAL/config/*
	/usr/local/include/CGAL/config/*
	/usr/include/CGAL/config/*
	$ENV{ProgramW6432}/CGAL*/include/CGAL/config/*
	$ENV{ProgramFiles}/CGAL*/include/CGAL/config/*
)
file(
	GLOB
	CGAL_INCLUDE_PATHS
	$ENV{CGALROOT}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/CGAL*/include
	$ENV{ProgramFiles}/CGAL*/include
)
file(
	GLOB
	CGAL_LIBRARY_PATHS
	$ENV{CGALROOT}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/CGAL*/lib
	$ENV{ProgramFiles}/CGAL*/lib
)

find_path(
	CGAL_COMPILER_INCLUDE_DIR
	NAMES
	CGAL/compiler_config.h
	HINTS
	${CGAL_COMPILER_INCLUDE_PATHS}
	${CGAL_INCLUDE_PATHS}
)

find_path(
	CGAL_INCLUDE_DIR
	NAMES
	CGAL/CORE/CORE.h
	HINTS
	${CGAL_INCLUDE_PATHS}
)

find_library(
	CGAL_LIBRARY_DEBUG
	NAMES
	CGALd
	cgal-vc100-mt-gd-4.1 cgal-vc100-mt-gd-4.0 cgal-vc100-mt-gd
	cgal-vc90-mt-gd-4.1 cgal-vc90-mt-gd-4.0 cgal-vc90-mt-gd
	cgal-vc80-mt-gd-4.1 cgal-vc80-mt-gd-4.0 cgal-vc80-mt-gd
	cgal-vc71-mt-gd-4.1 cgal-vc71-mt-gd-4.0 cgal-vc71-mt-gd
	HINTS
	${CGAL_LIBRARY_PATHS}
)

find_library(
	CGAL_LIBRARY_RELEASE
	NAMES
	CGAL
	cgal-vc100-mt-4.1 cgal-vc100-mt-4.0 cgal-vc100-mt
	cgal-vc90-mt-4.1 cgal-vc90-mt-4.0 cgal-vc90-mt
	cgal-vc80-mt-4.1 cgal-vc80-mt-4.0 cgal-vc80-mt
	cgal-vc71-mt-4.1 cgal-vc71-mt-4.0 cgal-vc71-mt
	HINTS
	${CGAL_LIBRARY_PATHS}
)

if(CGAL_LIBRARY_DEBUG AND NOT CGAL_LIBRARY_RELEASE)
	set(CGAL_LIBRARIES ${CGAL_LIBRARY_DEBUG})
endif(CGAL_LIBRARY_DEBUG AND NOT CGAL_LIBRARY_RELEASE)

if(CGAL_LIBRARY_RELEASE AND NOT CGAL_LIBRARY_DEBUG)
	set(CGAL_LIBRARIES ${CGAL_LIBRARY_RELEASE})
endif(CGAL_LIBRARY_RELEASE AND NOT CGAL_LIBRARY_DEBUG)

if(CGAL_LIBRARY_DEBUG AND CGAL_LIBRARY_RELEASE)
	set(CGAL_LIBRARIES debug ${CGAL_LIBRARY_DEBUG} optimized ${CGAL_LIBRARY_RELEASE})
endif(CGAL_LIBRARY_DEBUG AND CGAL_LIBRARY_RELEASE)

set(CGAL_INCLUDE_DIRS ${CGAL_INCLUDE_DIR} ${CGAL_COMPILER_INCLUDE_DIR})

find_package_handle_standard_args(
	CGAL
	DEFAULT_MSG
	CGAL_INCLUDE_DIRS
	CGAL_LIBRARIES
)

mark_as_advanced(
	CGAL_COMPILER_INCLUDE_DIR
	CGAL_INCLUDE_DIR
	CGAL_INCLUDE_DIRS
	CGAL_LIBRARIES
	CGAL_LIBRARY_DEBUG
	CGAL_LIBRARY_RELEASE
) 
