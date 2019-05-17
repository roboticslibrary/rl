cmake_minimum_required(VERSION 2.8.11)

project(3rdparty)

include(ExternalProject)

if(WIN32)
	ExternalProject_Add(
		patch
		URL https://downloads.sourceforge.net/project/gnuwin32/patch/2.5.9-7/patch-2.5.9-7-bin.zip
		#URL ${CMAKE_CURRENT_SOURCE_DIR}/patch-2.5.9-7-bin.zip
		URL_MD5 b9c8b31d62f4b2e4f1887bbb63e8a905
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
		INSTALL_COMMAND
		${CMAKE_COMMAND} -E copy <SOURCE_DIR>/bin/patch.exe ${CMAKE_CURRENT_BINARY_DIR}/patch.exe &&
		${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/patch.exe.manifest ${CMAKE_CURRENT_BINARY_DIR}/patch.exe.manifest
	)
	set(PATCH_EXECUTABLE ${CMAKE_CURRENT_BINARY_DIR}/patch.exe)
else()
	ExternalProject_Add(
		patch
		SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
		INSTALL_COMMAND ""
	)
	set(PATCH_EXECUTABLE patch)
endif()

add_subdirectory(coin3d/cpack.d)
add_subdirectory(coin3d/soanydata)
add_subdirectory(coin3d/sogui)

add_subdirectory(atidaq)
add_subdirectory(boost)
add_subdirectory(bullet)
add_subdirectory(eigen)
add_subdirectory(libiconv)
add_subdirectory(nlopt)
add_subdirectory(ode)
add_subdirectory(pqp)
add_subdirectory(coin3d/simage)
add_subdirectory(solid)
add_subdirectory(xz)
add_subdirectory(zlib)

add_subdirectory(libxml2)
add_subdirectory(coin3d/coin)

add_subdirectory(libxslt)
add_subdirectory(coin3d/soqt)

set(
	CPACK_INSTALL_CMAKE_PROJECTS
	"${CMAKE_CURRENT_BINARY_DIR}/atidaq/atidaq-prefix/src/atidaq-build;atidaq;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/bullet/bullet-prefix/src/bullet-build;bullet;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/coin3d/coin/coin-prefix/src/coin-build;coin;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/coin3d/simage/simage-prefix/src/simage-build;simage;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/coin3d/soqt/soqt-prefix/src/soqt-build;soqt;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/eigen/eigen-prefix/src/eigen-build;eigen;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/libiconv/libiconv-prefix/src/libiconv-build;libiconv;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/libxml2/libxml2-prefix/src/libxml2-build;libxml2;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/libxslt/libxslt-prefix/src/libxslt-build;libxslt;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/nlopt/nlopt-prefix/src/nlopt-build;nlopt;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/ode/ode-prefix/src/ode-build;ode;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/pqp/pqp-prefix/src/pqp-build;pqp;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/solid/solid-prefix/src/solid-build;solid;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/xz/xz-prefix/src/xz-build;xz;ALL;/"
	"${CMAKE_CURRENT_BINARY_DIR}/zlib/zlib-prefix/src/zlib-build;zlib;ALL;/"
)

set(
	CPACK_INSTALLED_DIRECTORIES
	${CMAKE_CURRENT_BINARY_DIR}/boost/boost-prefix/src/boost/boost ${CMAKE_INSTALL_INCLUDEDIR}/boost
)

set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 7)
set(CPACK_PACKAGE_VERSION_PATCH 0)

set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

if(MSVC)
	if(CMAKE_CXX_COMPILER_VERSION MATCHES "^([0-9]+)\\.([0-9])([0-9]*)\\.([0-9]+)\\.")
		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 18)
			math(EXPR LINKER_VERSION_MAJOR "${CMAKE_MATCH_1} - 5")
		else()
			math(EXPR LINKER_VERSION_MAJOR "${CMAKE_MATCH_1} - 6")
		endif()
		set(PLATFORM_TOOLSET_MAJOR ${LINKER_VERSION_MAJOR})
		set(PLATFORM_TOOLSET_MINOR ${CMAKE_MATCH_2})
		set(LINKER_VERSION "${LINKER_VERSION_MAJOR}.${CMAKE_MATCH_2}${CMAKE_MATCH_3}.${CMAKE_MATCH_4}")
	endif()
	if(CMAKE_CL_64)
		set(CPACK_PACKAGE_FILE_NAME "rl-3rdparty-${CPACK_PACKAGE_VERSION}-msvc-${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR}-x64")
		set(CPACK_PACKAGE_INSTALL_DIRECTORY "Robotics Library\\\\${CPACK_PACKAGE_VERSION}\\\\MSVC\\\\${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR}\\\\x64")
		set(CPACK_PACKAGE_NAME "Robotics Library ${CPACK_PACKAGE_VERSION} Third-Party Dependencies - MSVC ${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR} (x64)")
	else()
		set(CPACK_PACKAGE_FILE_NAME "rl-3rdparty-${CPACK_PACKAGE_VERSION}-msvc-${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR}-x86")
		set(CPACK_PACKAGE_INSTALL_DIRECTORY "Robotics Library\\\\${CPACK_PACKAGE_VERSION}\\\\MSVC\\\\${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR}\\\\x86")
		set(CPACK_PACKAGE_NAME "Robotics Library ${CPACK_PACKAGE_VERSION} Third-Party Dependencies - MSVC ${PLATFORM_TOOLSET_MAJOR}.${PLATFORM_TOOLSET_MINOR} (x86)")
	endif()
else()
	set(CPACK_PACKAGE_FILE_NAME "rl-3rdparty-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}")
	set(CPACK_PACKAGE_INSTALL_DIRECTORY "rl-${CPACK_PACKAGE_VERSION}")
	set(CPACK_PACKAGE_NAME "Robotics Library ${CPACK_PACKAGE_VERSION} Third-Party Dependencies")
endif()

set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://www.roboticslibrary.org/contact")
set(CPACK_DEBIAN_PACKAGE_NAME "librl-3rdparty")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_NSIS_DISPLAY_NAME ${CPACK_PACKAGE_NAME})
set(CPACK_NSIS_EXECUTABLES_DIRECTORY ${CMAKE_INSTALL_BINDIR})
set(CPACK_NSIS_HELP_LINK "https://www.roboticslibrary.org/contact")
set(CPACK_NSIS_INSTALLED_ICON_NAME Uninstall.exe)
set(CPACK_NSIS_MODIFY_PATH ON)
set(CPACK_NSIS_MUI_ICON ${CMAKE_CURRENT_SOURCE_DIR}/../robotics-library.ico)
set(CPACK_NSIS_MUI_UNIICON ${CMAKE_CURRENT_SOURCE_DIR}/../robotics-library.ico)
set(CPACK_NSIS_PACKAGE_NAME ${CPACK_PACKAGE_NAME})
set(CPACK_NSIS_URL_INFO_ABOUT "https://www.roboticslibrary.org/contact")
set(CPACK_PACKAGE_CONTACT "Robotics Library Team <roblib-developers@lists.sourceforge.net>")
set(CPACK_PACKAGE_VENDOR "Robotics Library")
set(CPACK_PROJECT_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/CPackProjectConfig.cmake)
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
set(CPACK_RPM_PACKAGE_NAME "rl-3rdparty")
set(CPACK_RPM_PACKAGE_SUMMARY "Robotics Library Third-Party Dependencies")
set(CPACK_RPM_PACKAGE_URL "https://www.roboticslibrary.org/contact")
set(CPACK_WIX_EXTENSIONS "WiXUtilExtension")
set(
	CPACK_WIX_EXTRA_SOURCES
	${CMAKE_CURRENT_BINARY_DIR}/WixExtra.wxs
	${CMAKE_CURRENT_SOURCE_DIR}/../cmake/WixUI_FeatureTreeNoLicense.wxs
)
set(CPACK_WIX_PATCH_FILE ${CMAKE_CURRENT_BINARY_DIR}/WixPatch.xml)
set(CPACK_WIX_PRODUCT_ICON ${CMAKE_CURRENT_SOURCE_DIR}/../robotics-library.ico)
set(CPACK_WIX_PROGRAM_MENU_FOLDER ${CPACK_PACKAGE_NAME})
set(CPACK_WIX_PROPERTY_ARPHELPLINK "https://www.roboticslibrary.org/contact")
set(CPACK_WIX_PROPERTY_ARPURLINFOABOUT "https://www.roboticslibrary.org/")
set(CPACK_WIX_ROOT_FEATURE_TITLE "Robotics Library Third-Party Dependencies")
set(CPACK_WIX_UI_BANNER ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/WixUIBanner.bmp)
set(CPACK_WIX_UI_DIALOG ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/WixUIDialog.bmp)
set(CPACK_WIX_UI_REF "WixUI_FeatureTreeNoLicense")

configure_file(../cmake/WixExtra.wxs.in ${CMAKE_CURRENT_BINARY_DIR}/WixExtra.wxs)
configure_file(../cmake/WixPatch.xml.in ${CMAKE_CURRENT_BINARY_DIR}/WixPatch.xml)

set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT system)
include(InstallRequiredSystemLibraries)

include(CPack)
