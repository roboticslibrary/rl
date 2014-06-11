cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
project(w32installercomplete)

include(ExternalProject)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install CACHE PATH "FOO install prefix" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(EXTERNAL_BINARIES_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rdparty)

option(make_thirdparty "Download and install third party dependencies (needs to be done in the first run)" OFF)
option(make_rl "Compile and install RL (needs to be added in the second run)" OFF)

if(${make_thirdparty})
externalproject_add(
	boost
	URL http://download.fortiss.org/public/rl/boost-1.52.0-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/boost
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/boost/include
	DESTINATION .
	COMPONENT boost
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/boost/lib
	DESTINATION .
	COMPONENT boost
#	FILES_MATCHING
#	PATTERN "*libboost_chrono*"
#	PATTERN "*libboost_date_time*"
#	PATTERN "*libboost_system*"
#	PATTERN "*libboost_thread*"
)
#TODO: install some needed static libs!

externalproject_add(
	bullet
	URL http://download.fortiss.org/public/rl/bullet-2.78-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/bullet
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/bullet/include
	DESTINATION .
	COMPONENT bullet
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/bullet/lib
	DESTINATION .
	COMPONENT bullet
)

externalproject_add(
	cgal
	URL http://download.fortiss.org/public/rl/CGAL-4.1-boost-1.52-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/cgal
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/cgal/bin
	DESTINATION .
	COMPONENT cgal
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/cgal/include
	DESTINATION .
	COMPONENT cgal
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/cgal/lib
	DESTINATION .
	COMPONENT cgal
)

externalproject_add(
	coin
	URL http://download.fortiss.org/public/rl/Coin-3.1.3-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/coin
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/coin/bin
	DESTINATION .
	COMPONENT coin
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/coin/include
	DESTINATION .
	COMPONENT coin
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/coin/lib
	DESTINATION .
	COMPONENT coin
)

externalproject_add(
	eigen
	URL http://download.fortiss.org/public/rl/eigen-3.1.2.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/eigen
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/eigen/eigen3/Eigen 
	DESTINATION include
	COMPONENT eigen
)

externalproject_add(
	xml2
	URL http://download.fortiss.org/public/rl/libxml2-2.7.8-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/xml2
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/xml2/bin
	DESTINATION .
	COMPONENT xml2
	PATTERN "*.exe" EXCLUDE
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/xml2/include
	DESTINATION .
	COMPONENT xml2
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/xml2/lib
	DESTINATION .
	COMPONENT xml2
)

externalproject_add(
	ode
	URL http://download.fortiss.org/public/rl/ode-0.11.1-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/ode
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/ode/include
	DESTINATION .
	COMPONENT ode
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/ode/lib
	DESTINATION .
	COMPONENT ode
)
install(FILES ${EXTERNAL_BINARIES_DIR}/ode/lib/ReleaseSingleDLL/ode_single.dll
	DESTINATION bin
	COMPONENT ode
)
install(FILES ${EXTERNAL_BINARIES_DIR}/ode/lib/ReleaseSingleDLL/ode_single.lib
	DESTINATION lib
	COMPONENT ode
)
#TODO: install some needed dlls to bin

externalproject_add(
	pqp
	URL http://download.fortiss.org/public/rl/pqp-1.3-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/pqp
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/pqp/include
	DESTINATION .
	COMPONENT pqp
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/pqp/lib
	DESTINATION .
	COMPONENT pqp
)

externalproject_add(
	simage
	URL http://download.fortiss.org/public/rl/simage-1.7.0-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/simage
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/simage/bin
	DESTINATION .
	COMPONENT simage
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/simage/include
	DESTINATION .
	COMPONENT simage
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/simage/lib
	DESTINATION .
	COMPONENT simage
)

externalproject_add(
	solid
	URL http://download.fortiss.org/public/rl/solid-3.5.6-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/solid
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/solid/bin
	DESTINATION .
	COMPONENT solid
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/solid/include
	DESTINATION .
	COMPONENT solid
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/solid/lib
	DESTINATION .
	COMPONENT solid
)

externalproject_add(
	soqt
	URL http://download.fortiss.org/public/rl/SoQt-1.5.0-Coin-3.1.3-Qt-4.7.3-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/soqt
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/soqt/bin
	DESTINATION .
	COMPONENT soqt
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/soqt/include
	DESTINATION .
	COMPONENT soqt
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/soqt/lib
	DESTINATION .
	COMPONENT soqt
)

externalproject_add(
	zlib
	URL http://download.fortiss.org/public/rl/zlib-1.2.5-msvc10-x86.zip
	SOURCE_DIR ${EXTERNAL_BINARIES_DIR}/zlib
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/zlib/bin
	DESTINATION .
	COMPONENT zlib
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/zlib/include
	DESTINATION .
	COMPONENT zlib
)
install(DIRECTORY ${EXTERNAL_BINARIES_DIR}/zlib/lib
	DESTINATION .
	COMPONENT zlib
)
endif(${make_thirdparty})

set(VERSION_MAJOR 0)
set(VERSION_MINOR 6)
set(VERSION_PATCH 2)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

find_path(
	RL_SOURCE_DIR
	src/rl/plan/Rrt.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/../
	${CMAKE_CURRENT_SOURCE_DIR}/../../
)
if(${make_rl})
set(RL_MAIN_DIR "$ENV{ProgramFiles}/rl-${VERSION}")
set(ENV{HOME} ${RL_MAIN_DIR})
set(ENV{Boost_DIR} ${RL_MAIN_DIR})
set(ENV{BOOST_ROOT} ${RL_MAIN_DIR})
set(USE_STATIC_BOOST TRUE)
set(Boost_USE_STATIC_LIBS TRUE)
message(STATUS "Set ENV{HOME} to $ENV{HOME} in order to link correct dependencies")


set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "")
message(STATUS "CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}")
add_subdirectory(${RL_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/rl)
#FIXME: Hack to overwrite wrong rl-config-version.cmake 
# Update: should be ok in CMake 2.8.11
#add_custom_command(TARGET MyTarget POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy
#    ${CMAKE_SOURCE_DIR}/rl-config-version.cmake 
#	${CMAKE_CURRENT_BINARY_DIR}/rl/rl-config-version.cmake)
#add_custom_target(MyTarget)
#set(CPACK_PACKAGE_EXECUTABLES 
#	"rlCoachKin" "RLCoachKin Denavit-Hartenberg Kinematics Viewer"
#	"rlCoachMdl" "RLCoachMdl Kinematics Viewer"
#	"rlPlanDemo" "Robotics Library Path Planning Demo"
#	"rlCollisionDemo" "Robotics Library Collision Detection Demo"
#)
message(STATUS "CPACK_NSIS_EXTRA_INSTALL_COMMANDS ${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}")
endif(${make_rl})

#set(CPACK_PACKAGE_CONTACT "Robotics Library Team <roblib-developers@lists.sourceforge.net>")
#set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Robotics Library is a self-contained C++ library for robot kinematics, motion planning, and control.")
#set(CPACK_PACKAGE_DISPLAY_NAME "Robotics Library ${VERSION}")
if(${make_rl})
	set(CPACK_PACKAGE_NAME "rl")
else(${make_rl})
	set(CPACK_PACKAGE_NAME "rl-3rdparty-dependencies")
endif(${make_rl})
set(CPACK_PACKAGE_VENDOR "http://www.roboticslibrary.org/")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "rl-${VERSION}")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_BINARY_DIR}/LICENSES.txt)
#set(CPACK_NSIS_MENU_LINKS 
#	"http://www.roboticslibrary.org/" "Robotics Library Website"
#	"http://sourceforge.net/projects/roblib/" "Robotics Library Sourceforge Project"
#)

include(CPack)
if(${make_rl})
cpack_add_component(libraries 
	DISPLAY_NAME "Libraries" GROUP "RL" 
	DESCRIPTION "RL Libraries, all components"
	INSTALL_TYPES "dev"
)
cpack_add_component(headers 
	DISPLAY_NAME "Headers" GROUP "RL" 
	DESCRIPTION "Headers, needed for developers using RL"
	#DEPENDS libraries 
	INSTALL_TYPES "dev"
)
cpack_add_component(demos 
	DISPLAY_NAME "Demos" GROUP "RL" 
	DESCRIPTION "Demo programs and tools, including the robot simulator Coach"
	#DEPENDS libraries 
	INSTALL_TYPES "dev"
)
cpack_add_component(examples 
	DISPLAY_NAME "Examples" GROUP "RL" 
	DESCRIPTION "Example robot kinematics and geometry files"
	#DEPENDS LIBRARIES 
	INSTALL_TYPES "dev"
)
cpack_add_component(extras 
	DISPLAY_NAME "VRML Viewer" GROUP "RL" 
	DESCRIPTION "Viewer for VRML and Open Inventor files"
	#DEPENDS LIBRARIES 
	INSTALL_TYPES "dev"
)
cpack_add_component_group("RL" DISPLAY_NAME "Robotics Library" EXPANDED)
endif(${make_rl})
if(${make_thirdparty})
cpack_add_component(boost DISPLAY_NAME "Boost" GROUP "THIRDPARTY" DESCRIPTION "Boost C++ Libraries 1.52 msvc10-x86" INSTALL_TYPES "dev")
cpack_add_component(bullet DISPLAY_NAME "Bullet Physics Library" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(cgal DISPLAY_NAME "CGAL" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(coin DISPLAY_NAME "COIN" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(eigen DISPLAY_NAME "Eigen" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(xml2 DISPLAY_NAME "LibXML2" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(ode DISPLAY_NAME "ODE" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(pqp DISPLAY_NAME "PQP" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(simage DISPLAY_NAME "Coin 3D simage" GROUP "THIRDPARTY" DESCRIPTION "Coin 3D simage https://bitbucket.org/Coin3D/simage" INSTALL_TYPES "dev")
cpack_add_component(solid DISPLAY_NAME "SOLID" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(soqt DISPLAY_NAME "SoQT" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component(zlib DISPLAY_NAME "ZLib" GROUP "THIRDPARTY" INSTALL_TYPES "dev")
cpack_add_component_group("THIRDPARTY" DISPLAY_NAME "Third Party Dependency Libraries" EXPANDED)

cpack_add_install_type("dev" DISPLAY_NAME "Libraries for development (recommended)")

endif(${make_thirdparty})

file(GLOB LICENSE_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdpartylicenses/*.txt
)
message(STATUS "License files: ${LICENSE_FILES}")
message(STATUS "RL_SOURCE_DIR: ${RL_SOURCE_DIR}")
file(READ ${RL_SOURCE_DIR}/LICENSE RL_LIC_TEXT)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/LICENSES.txt "All licenses:\n\n\nRobotics Library License:\n\n${RL_LIC_TEXT}\n\n\n")
foreach(LIC ${LICENSE_FILES})
	file(READ ${LIC} LIC_TEXT)
	get_filename_component(LIC_NAME ${LIC} NAME)
	file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/LICENSES.txt "Licence for ${LIC_NAME}:\n\n${LIC_TEXT}\n\n")
endforeach(LIC ${LICENSE_FILES})
