include(ExternalProject)

ExternalProject_Add(
	soanydata
	#GIT_REPOSITORY https://github.com/coin3d/soanydata
	#GIT_TAG 3ff6e9203fbb0cc08a2bdf209212b7ef4d78a1f2
	URL https://github.com/coin3d/soanydata/archive/3ff6e9203fbb0cc08a2bdf209212b7ef4d78a1f2.zip
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/soanydata-3ff6e9203fbb0cc08a2bdf209212b7ef4d78a1f2.zip
	URL_MD5 17c8b33000f58fdb99d943e559d8fa21
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
