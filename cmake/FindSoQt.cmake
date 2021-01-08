include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(
	SoQt_INCLUDE_DIR
	NAMES Inventor/Qt/SoQt.h
	PATH_SUFFIXES Coin4 Coin3 Coin2
)
find_library(
	SoQt_LIBRARY_DEBUG
	NAMES SoQtd SoQt1d
)
find_library(
	SoQt_LIBRARY_RELEASE
	NAMES SoQt SoQt1
)
select_library_configurations(SoQt)

if(SoQt_INCLUDE_DIR AND EXISTS "${SoQt_INCLUDE_DIR}/Inventor/Qt/SoQtBasic.h")
	file(STRINGS "${SoQt_INCLUDE_DIR}/Inventor/Qt/SoQtBasic.h" _SoQt_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+SOQT_VERSION[\t ]+\"[^\"]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+SOQT_VERSION[\t ]+\"([^\"]*)\".*" "\\1" SoQt_VERSION "${_SoQt_VERSION_DEFINE}")
	file(STRINGS "${SoQt_INCLUDE_DIR}/Inventor/Qt/SoQtBasic.h" _SoQt_GUI_TOOLKIT_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+GUI_TOOLKIT_VERSION[\t ]+\"[0-9][0-9]*\".*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+GUI_TOOLKIT_VERSION[\t ]+\"([0-9])[0-9]*\".*" "\\1" SoQt_GUI_TOOLKIT_VERSION_MAJOR "${_SoQt_GUI_TOOLKIT_VERSION_DEFINE}")
	if(NOT SoQt_GUI_TOOLKIT_VERSION_MAJOR STREQUAL "")
		set(SoQt_HAVE_QT${SoQt_GUI_TOOLKIT_VERSION_MAJOR} ON)
	endif()
	unset(_SoQt_VERSION_DEFINE)
	unset(_SoQt_GUI_TOOLKIT_VERSION_DEFINE)
endif()

set(SoQt_DEFINITIONS -DSOQT_DLL)
set(SoQt_INCLUDE_DIRS ${SoQt_INCLUDE_DIR})
set(SoQt_LIBRARIES ${SoQt_LIBRARY})

find_package_handle_standard_args(
	SoQt
	FOUND_VAR SoQt_FOUND
	REQUIRED_VARS SoQt_INCLUDE_DIR SoQt_LIBRARY
	VERSION_VAR SoQt_VERSION
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
	set_target_properties(SoQt::SoQt PROPERTIES INTERFACE_COMPILE_DEFINITIONS "SOQT_DLL")
	set_target_properties(SoQt::SoQt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${SoQt_INCLUDE_DIRS}")
endif()

mark_as_advanced(SoQt_DEFINITIONS)
mark_as_advanced(SoQt_INCLUDE_DIR)
