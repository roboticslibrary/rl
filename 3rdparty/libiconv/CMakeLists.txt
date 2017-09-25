include(ExternalProject)

if(WIN32)
	set(PATCH_BINARY "--binary")
endif()

ExternalProject_Add(
	libiconv
	DEPENDS patch
	#GIT_REPOSITORY git://git.savannah.gnu.org/libiconv.git
	#GIT_TAG v1.15
	URL http://ftp.gnu.org/pub/gnu/libiconv/libiconv-1.15.tar.gz
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/libiconv-1.15.tar.gz
	URL_MD5 ace8b5f2db42f7b3b3057585e80d9808
	PATCH_COMMAND
	${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt.in <SOURCE_DIR>/CMakeLists.txt &&
	${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake.in <SOURCE_DIR>/config.h.cmake.in &&
	${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/libiconv-config.cmake.in <SOURCE_DIR>/libiconv-config.cmake.in &&
	${PATCH_EXECUTABLE} -p1 -t -N ${PATCH_BINARY} < ${CMAKE_CURRENT_SOURCE_DIR}/libiconv-1.15.patch
	CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CFG_INTDIR=${CMAKE_CFG_INTDIR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} -DCMAKE_TRY_COMPILE_CONFIGURATION=${CMAKE_TRY_COMPILE_CONFIGURATION}
	INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
)
