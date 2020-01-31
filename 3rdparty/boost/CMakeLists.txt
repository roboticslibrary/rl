cmake_minimum_required(VERSION 2.8.11)

project(boost)

include(ExternalProject)
include(GNUInstallDirs)

ExternalProject_Add(
	boost
	URL https://dl.bintray.com/boostorg/release/1.72.0/source/boost_1_72_0.7z
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/boost_1_72_0.7z
	URL_MD5 cbf2f2abf8cecf69210b60478bc51109
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	BUILD_IN_SOURCE 1
	INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/boost ${CMAKE_INSTALL_FULL_INCLUDEDIR}/boost
)
