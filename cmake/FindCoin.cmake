include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	Coin_INCLUDE_DIR
	NAMES
	Inventor/So.h
	PATH_SUFFIXES
	Coin4
	Coin3
	Coin2
)

mark_as_advanced(Coin_INCLUDE_DIR)

set(Coin_INCLUDE_DIRS ${Coin_INCLUDE_DIR})

find_path(
	Coin_FOREIGN_FILES_INCLUDE_DIR
	NAMES
	ForeignFiles/SoForeignFileKit.h
	PATH_SUFFIXES
	Inventor/annex
	Coin4/Inventor/annex
	Coin3/Inventor/annex
	Coin2/Inventor/annex
)

mark_as_advanced(Coin_FOREIGN_FILES_INCLUDE_DIR)

if(Coin_FOREIGN_FILES_INCLUDE_DIR)
	list(APPEND Coin_INCLUDE_DIRS ${Coin_FOREIGN_FILES_INCLUDE_DIR})
endif()

find_library(
	Coin_LIBRARY_DEBUG
	NAMES
	Coind Coin4d Coin3d Coin2d
)

find_library(
	Coin_LIBRARY_RELEASE
	NAMES
	Coin Coin4 Coin3 Coin2
)

select_library_configurations(Coin)

set(Coin_DEFINITIONS -DCOIN_DLL)

mark_as_advanced(Coin_DEFINITIONS)

if(Coin_INCLUDE_DIR AND EXISTS "${Coin_INCLUDE_DIR}/Inventor/C/basic.h")
	file(STRINGS "${Coin_INCLUDE_DIR}/Inventor/C/basic.h" _Coin_VERSION_DEFINE REGEX "#define[\t ]+COIN_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "#define[\t ]+COIN_VERSION[\t ]+\"([^\"]*)\".*" "\\1" Coin_VERSION "${_Coin_VERSION_DEFINE}")
	unset(_Coin_VERSION_DEFINE)
endif()

find_package_handle_standard_args(
	Coin
	FOUND_VAR Coin_FOUND
	REQUIRED_VARS Coin_INCLUDE_DIRS Coin_LIBRARIES
	VERSION_VAR Coin_VERSION
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
