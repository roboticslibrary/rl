cmake_minimum_required(VERSION 2.8.11)

project(boost)

include(ExternalProject)
include(GNUInstallDirs)

ExternalProject_Add(
	boost
	URL https://dl.bintray.com/boostorg/release/1.65.1/source/boost_1_65_1.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/boost_1_65_1.tar.bz2
	URL_MD5 41d7542ce40e171f3f7982aff008ff0d
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/boost ${CMAKE_INSTALL_FULL_INCLUDEDIR}/boost
)
