include(ExternalProject)

ExternalProject_Add(
	sogui
	#HG_REPOSITORY https://bitbucket.org/Coin3D/sogui
	#HG_TAG default
	URL https://bitbucket.org/Coin3D/sogui/get/default.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/default.tar.bz2
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)

list(INSERT CMAKE_PREFIX_PATH 0 ${CMAKE_INSTALL_PREFIX})

ExternalProject_Add(
	soqt
	DEPENDS sogui coin
	#HG_REPOSITORY https://bitbucket.org/Coin3D/soqt
	#HG_TAG default
	URL https://bitbucket.org/Coin3D/soqt/get/default.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/default.tar.bz2
	PATCH_COMMAND
	${CMAKE_COMMAND} -E rename ${CMAKE_CURRENT_BINARY_DIR}/sogui-prefix/src/sogui <SOURCE_DIR>/src/Inventor/Qt/common
	CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CFG_INTDIR=${CMAKE_CFG_INTDIR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} -DCMAKE_TRY_COMPILE_CONFIGURATION=${CMAKE_TRY_COMPILE_CONFIGURATION}
	INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
)
