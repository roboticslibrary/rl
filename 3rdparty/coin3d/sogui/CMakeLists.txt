include(ExternalProject)

ExternalProject_Add(
	sogui
	#GIT_REPOSITORY https://github.com/coin3d/sogui
	#GIT_TAG fb79af47cff89f0f3657501601a7ea5c11968b17
	URL https://github.com/coin3d/sogui/archive/fb79af47cff89f0f3657501601a7ea5c11968b17.zip
	#URL ${CMAKE_CURRENT_SOURCE_DIR}/sogui-fb79af47cff89f0f3657501601a7ea5c11968b17
	URL_MD5 2352490aadfd6612cb5e8aa7f619f12e
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
