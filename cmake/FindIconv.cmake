include(CheckCSourceRuns)
include(CMakePushCheckState)
include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	Iconv_INCLUDE_DIRS
	NAMES
	iconv.h
)

mark_as_advanced(Iconv_INCLUDE_DIRS)

find_library(
	Iconv_LIBRARY_DEBUG
	NAMES
	libiconvd iconvd
)

find_library(
	Iconv_LIBRARY_RELEASE
	NAMES
	libiconv iconv
)

select_library_configurations(Iconv)

if(Iconv_INCLUDE_DIRS AND NOT Iconv_LIBRARIES)
	cmake_push_check_state(RESET)
	set(CMAKE_REQUIRED_INCLUDES ${Iconv_INCLUDE_DIRS})
	check_c_source_runs("
		#include <iconv.h>
		int main() { iconv_t ic = iconv_open(\"to\", \"from\"); return 0; }
	" Iconv_IS_BUILT_IN)
	cmake_pop_check_state()
	
	if(Iconv_IS_BUILT_IN)
		unset(Iconv_LIBRARIES)
	endif()
endif()

if(Iconv_INCLUDE_DIRS AND EXISTS "${Iconv_INCLUDE_DIRS}/iconv.h")
	file(STRINGS "${Iconv_INCLUDE_DIRS}/iconv.h" _Iconv_VERSION_DEFINE REGEX "#define[\t ]+_LIBICONV_VERSION[\t ]+0x[0-9A-F][0-9A-F][0-9A-F][0-9A-F].*")
	string(REGEX REPLACE "#define[\t ]+_LIBICONV_VERSION[\t ]+0x([0-9A-F][0-9A-F])[0-9A-F][0-9A-F].*" "\\1" _Iconv_VERSION_MAJOR_HEXADECIMAL "${_Iconv_VERSION_DEFINE}")
	string(REGEX REPLACE "#define[\t ]+_LIBICONV_VERSION[\t ]+0x[0-9A-F][0-9A-F]([0-9A-F][0-9A-F]).*" "\\1" _Iconv_VERSION_MINOR_HEXADECIMAL "${_Iconv_VERSION_DEFINE}")
	
	if(NOT _Iconv_VERSION_MAJOR_HEXADECIMAL STREQUAL "" AND NOT _Iconv_VERSION_MINOR_HEXADECIMAL STREQUAL "")
		math(EXPR Iconv_VERSION_MAJOR "0x${_Iconv_VERSION_MAJOR_HEXADECIMAL}" OUTPUT_FORMAT DECIMAL)
		math(EXPR Iconv_VERSION_MINOR "0x${_Iconv_VERSION_MINOR_HEXADECIMAL}" OUTPUT_FORMAT DECIMAL)
		set(Iconv_VERSION "${Iconv_VERSION_MAJOR}.${Iconv_VERSION_MINOR}")
	endif()
	
	unset(_Iconv_VERSION_DEFINE)
	unset(_Iconv_VERSION_MAJOR_HEXADECIMAL)
	unset(_Iconv_VERSION_MINOR_HEXADECIMAL)
endif()

if(Iconv_IS_BUILT_IN)
	find_package_handle_standard_args(
		Iconv
		FOUND_VAR Iconv_FOUND
		REQUIRED_VARS Iconv_INCLUDE_DIRS
		VERSION_VAR Iconv_VERSION
	)
else()
	find_package_handle_standard_args(
		Iconv
		FOUND_VAR Iconv_FOUND
		REQUIRED_VARS Iconv_INCLUDE_DIRS Iconv_LIBRARIES
		VERSION_VAR Iconv_VERSION
	)
endif()

if(Iconv_FOUND AND NOT TARGET Iconv::Iconv)
	add_library(Iconv::Iconv UNKNOWN IMPORTED)
	
	if(Iconv_LIBRARY_RELEASE)
		set_property(TARGET Iconv::Iconv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Iconv::Iconv PROPERTIES IMPORTED_LOCATION_RELEASE "${Iconv_LIBRARY_RELEASE}")
	endif()
	
	if(Iconv_LIBRARY_DEBUG)
		set_property(TARGET Iconv::Iconv APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Iconv::Iconv PROPERTIES IMPORTED_LOCATION_DEBUG "${Iconv_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Iconv::Iconv PROPERTIES
		INTERFACE_COMPILE_DEFINITIONS "${Iconv_DEFINITIONS}"
		INTERFACE_INCLUDE_DIRECTORIES "${Iconv_INCLUDE_DIRS}"
	)
endif()
