include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	cifX_INCLUDE_DIRS
	NAMES
	cifXUser.h
	PATHS
	"$ENV{ProgramW6432}/cifX Device Driver/SDK/includes"
	"$ENV{ProgramFiles}/cifX Device Driver/SDK/includes"
	PATH_SUFFIXES
	cifx
)

mark_as_advanced(cifX_INCLUDE_DIRS)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(cifX_ARCHITECTURE "x64")
else()
	set(cifX_ARCHITECTURE "x86")
endif()

find_library(
	cifX_LIBRARY_RELEASE
	NAMES
	cifx cifx32dll
	PATHS
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
