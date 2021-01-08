include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(CIFX_ARCHITECTURE "x64")
else()
	set(CIFX_ARCHITECTURE "x86")
endif()

find_path(
	CIFX_INCLUDE_DIR
	NAMES cifXUser.h
	PATHS
		"$ENV{ProgramW6432}/cifX Device Driver/SDK/includes"
		"$ENV{ProgramFiles}/cifX Device Driver/SDK/includes"
	PATH_SUFFIXES cifx
)
find_library(
	CIFX_LIBRARY_RELEASE
	NAMES cifx cifx32dll
	PATHS
		"$ENV{ProgramW6432}/cifX Device Driver/SDK/libs/${CIFX_ARCHITECTURE}"
		"$ENV{ProgramFiles}/cifX Device Driver/SDK/libs/${CIFX_ARCHITECTURE}"
)
select_library_configurations(CIFX)

set(CIFX_INCLUDE_DIRS ${CIFX_INCLUDE_DIR})
set(CIFX_LIBRARIES ${CIFX_LIBRARY})

find_package_handle_standard_args(
	cifX
	FOUND_VAR cifX_FOUND
	REQUIRED_VARS CIFX_INCLUDE_DIR CIFX_LIBRARY
)

if(cifX_FOUND AND NOT TARGET cifX::cifX)
	add_library(cifX::cifX UNKNOWN IMPORTED)
	if(CIFX_LIBRARY_RELEASE)
		set_property(TARGET cifX::cifX APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(cifX::cifX PROPERTIES IMPORTED_LOCATION_RELEASE "${CIFX_LIBRARY_RELEASE}")
	endif()
	set_target_properties(cifX::cifX PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CIFX_INCLUDE_DIRS}")
endif()

mark_as_advanced(CIFX_INCLUDE_DIR)
