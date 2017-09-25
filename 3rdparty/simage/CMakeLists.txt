cmake_minimum_required(VERSION 2.8.11)

project(simage)

include(ExternalProject)

ExternalProject_Add(
	simage
	#HG_REPOSITORY https://bitbucket.org/Coin3D/simage
	#HG_TAG default
	URL https://bitbucket.org/Coin3D/simage/get/default.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/default.tar.bz2
	CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CFG_INTDIR=${CMAKE_CFG_INTDIR} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} -DCMAKE_TRY_COMPILE_CONFIGURATION=${CMAKE_TRY_COMPILE_CONFIGURATION}
	INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
)
