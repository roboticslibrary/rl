include(CheckCXXSourceRuns)
include(CMakePushCheckState)
include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

if(NOT Bullet_FIND_COMPONENTS)
	set(Bullet_FIND_COMPONENTS BulletCollision BulletDynamics BulletSoftBody LinearMath)
	foreach(component IN LISTS Bullet_FIND_COMPONENTS)
		set(Bullet_FIND_REQUIRED_${component} ON)
	endforeach()
endif()

find_path(
	BULLET_INCLUDE_DIR
	NAMES btBulletCollisionCommon.h
	PATH_SUFFIXES bullet
)

foreach(component IN LISTS Bullet_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	find_library(
		BULLET_${COMPONENT}_LIBRARY_DEBUG
		NAMES ${component}-float64_Debug ${component}_Debug
	)
	find_library(
		BULLET_${COMPONENT}_LIBRARY_RELEASE
		NAMES ${component}-float64 ${component}
	)
	select_library_configurations(BULLET_${COMPONENT})
endforeach()

if(BULLET_INCLUDE_DIR AND BULLET_BULLETCOLLISION_LIBRARY AND BULLET_LINEARMATH_LIBRARY)
	cmake_push_check_state(RESET)
	set(CMAKE_REQUIRED_DEFINITIONS -DBT_USE_DOUBLE_PRECISION)
	set(CMAKE_REQUIRED_INCLUDES ${BULLET_INCLUDE_DIR})
	set(CMAKE_REQUIRED_LIBRARIES ${BULLET_BULLETCOLLISION_LIBRARY} ${BULLET_LINEARMATH_LIBRARY})
	check_cxx_source_runs("
		#include <btBulletCollisionCommon.h>
		int main()
		{
			btVector3 boxHalfExtents(1, -2, 3);
			btBoxShape box(boxHalfExtents);
			btVector3 boxHalfExtentsWithMargin = box.getHalfExtentsWithMargin();
			return !btFuzzyZero(boxHalfExtents.distance(boxHalfExtentsWithMargin));
		}
	" _BULLET_DOUBLE_PRECISION)
	if(_BULLET_DOUBLE_PRECISION)
		set(BULLET_DEFINITIONS -DBT_USE_DOUBLE_PRECISION)
		set(_BULLET_INTERFACE_COMPILE_DEFINITIONS "BT_USE_DOUBLE_PRECISION")
	endif()
	unset(_BULLET_DOUBLE_PRECISION)
	cmake_pop_check_state()
endif()

if(BULLET_INCLUDE_DIR AND EXISTS "${BULLET_INCLUDE_DIR}/LinearMath/btScalar.h")
	file(STRINGS "${BULLET_INCLUDE_DIR}/LinearMath/btScalar.h" _BULLET_VERSION_DEFINE REGEX "[\t ]*#define[\t ]+BT_BULLET_VERSION[\t ]+[0-9]+.*")
	string(REGEX REPLACE "[\t ]*#define[\t ]+BT_BULLET_VERSION[\t ]+([0-9])[0-9][0-9].*" "\\1" BULLET_VERSION_MAJOR "${_BULLET_VERSION_DEFINE}")
	string(REGEX REPLACE "[\t ]*#define[\t ]+BT_BULLET_VERSION[\t ]+[0-9]([0-9][0-9]).*" "\\1" BULLET_VERSION_MINOR "${_BULLET_VERSION_DEFINE}")
	if(NOT BULLET_VERSION_MAJOR STREQUAL "" AND NOT BULLET_VERSION_MINOR STREQUAL "")
		set(BULLET_VERSION "${BULLET_VERSION_MAJOR}.${BULLET_VERSION_MINOR}")
	endif()
	unset(_BULLET_VERSION_DEFINE)
endif()

set(BULLET_INCLUDE_DIRS ${BULLET_INCLUDE_DIR})
unset(BULLET_LIBRARIES)

foreach(component IN LISTS Bullet_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(BULLET_${COMPONENT}_LIBRARY)
		set(Bullet_${component}_FOUND ON)
		list(APPEND BULLET_LIBRARIES ${BULLET_${COMPONENT}_LIBRARY})
	endif()
endforeach()

find_package_handle_standard_args(
	Bullet
	FOUND_VAR Bullet_FOUND
	REQUIRED_VARS BULLET_INCLUDE_DIR
	VERSION_VAR BULLET_VERSION
	HANDLE_COMPONENTS
)

foreach(component IN LISTS Bullet_FIND_COMPONENTS)
	string(TOUPPER ${component} COMPONENT)
	if(Bullet_${component}_FOUND AND NOT TARGET Bullet::${component})
		add_library(Bullet::${component} UNKNOWN IMPORTED)
		if(BULLET_${COMPONENT}_LIBRARY_RELEASE)
			set_property(TARGET Bullet::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
			set_target_properties(Bullet::${component} PROPERTIES IMPORTED_LOCATION_RELEASE "${BULLET_${COMPONENT}_LIBRARY_RELEASE}")
		endif()
		if(BULLET_${COMPONENT}_LIBRARY_DEBUG)
			set_property(TARGET Bullet::${component} APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
			set_target_properties(Bullet::${component} PROPERTIES IMPORTED_LOCATION_DEBUG "${BULLET_${COMPONENT}_LIBRARY_DEBUG}")
		endif()
		set_target_properties(Bullet::${component} PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${_BULLET_INTERFACE_COMPILE_DEFINITIONS}")
		set_target_properties(Bullet::${component} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}")
	endif()
endforeach()

mark_as_advanced(BULLET_DEFINITIONS)
mark_as_advanced(BULLET_INCLUDE_DIR)
unset(_BULLET_INTERFACE_COMPILE_DEFINITIONS)
