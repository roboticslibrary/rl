include(ExternalProject)

ExternalProject_Add(
	cpack.d
	#GIT_REPOSITORY https://bitbucket.org/Coin3D/cpack.d
	#GIT_TAG b9ddb8116dd1bf98f2b1c858693c8b55cb80cad5
	URL https://github.com/coin3d/cpack.d/archive/b9ddb8116dd1bf98f2b1c858693c8b55cb80cad5.zip
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/cpack.d-b9ddb8116dd1bf98f2b1c858693c8b55cb80cad5.zip
	URL_MD5 35a40474754869bf86693349d00f49e2
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
