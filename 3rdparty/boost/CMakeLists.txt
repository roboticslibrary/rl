cmake_minimum_required(VERSION 2.8.11)

project(boost)

include(ExternalProject)
include(GNUInstallDirs)

ExternalProject_Add(
	boost
	URL https://dl.bintray.com/boostorg/release/1.66.0/source/boost_1_66_0.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/boost_1_66_0.tar.bz2
	URL_MD5 b2dfbd6c717be4a7bb2d88018eaccf75
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/boost ${CMAKE_INSTALL_FULL_INCLUDEDIR}/boost
)
