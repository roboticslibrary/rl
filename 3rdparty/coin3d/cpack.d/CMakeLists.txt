include(ExternalProject)

ExternalProject_Add(
	cpack.d
	#HG_REPOSITORY https://bitbucket.org/Coin3D/cpack.d
	#HG_TAG default
	URL https://bitbucket.org/Coin3D/cpack.d/get/default.tar.bz2
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/default.tar.bz2
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
