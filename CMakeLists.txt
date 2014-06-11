cmake_minimum_required(VERSION 2.8.11)

project(rl)

enable_testing()

include(CMakeDependentOption)
include(GNUInstallDirs)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 6)
set(VERSION_PATCH 2)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)

set(Boost_ADDITIONAL_VERSIONS "1.56.0" "1.56" "1.55.0" "1.55" "1.54.0" "1.54" "1.53.0" "1.53" "1.52.0" "1.52" "1.51.0" "1.51" "1.50.0" "1.50" "1.49.0" "1.49" "1.48.0" "1.48" "1.47.0" "1.47")

option(BUILD_DEMOS "Build demos" TRUE)
option(BUILD_EXTRAS "Build extras" TRUE)
option(BUILD_RL_MATH "Build RL::MATH" TRUE)
option(BUILD_RL_UTIL "Build RL::UTIL" TRUE)
option(BUILD_RL_XML "Build RL::XML" TRUE)
option(BUILD_TESTS "Build tests" TRUE)

cmake_dependent_option(BUILD_RL_HAL "Build RL::HAL" ON "BUILD_RL_MATH;BUILD_RL_UTIL" OFF)
cmake_dependent_option(BUILD_RL_KIN "Build RL::KIN" ON "BUILD_RL_MATH;BUILD_RL_XML" OFF)
cmake_dependent_option(BUILD_RL_MDL "Build RL::MDL" ON "BUILD_RL_MATH;BUILD_RL_XML" OFF)
cmake_dependent_option(BUILD_RL_SG "Build RL::SG" ON "BUILD_RL_MATH;BUILD_RL_UTIL;BUILD_RL_XML" OFF)

cmake_dependent_option(BUILD_RL_PLAN "Build RL::PLAN" ON "BUILD_RL_KIN;BUILD_RL_MATH;BUILD_RL_MDL;BUILD_RL_SG;BUILD_RL_UTIL;BUILD_RL_XML" OFF)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
	add_definitions(-DEIGEN_DONT_ALIGN)
endif(CMAKE_SIZEOF_VOID_P EQUAL 4)

if(WIN32)
	add_definitions(
		-D_CRT_SECURE_NO_WARNINGS
		-D_SCL_SECURE_NO_WARNINGS
		-D_USE_MATH_DEFINES
		-D_WIN32_WINNT=0x400
		-DNOMINMAX
	)
endif(WIN32)

include_directories(${rl_SOURCE_DIR}/src)

if(BUILD_RL_MATH)
	add_subdirectory(src/rl/math)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlDenavitHartenbergDemo)
		add_subdirectory(demos/rlInterpolatorDemo)
		add_subdirectory(demos/rlKalmanDemo)
		add_subdirectory(demos/rlPcaDemo)
		add_subdirectory(demos/rlQuaternionDemo)
		#add_subdirectory(demos/rlRodriguesDemo)
		add_subdirectory(demos/rlSpatialDemo)
		add_subdirectory(demos/rlTrapezoidalVelocityDemo)
	endif(BUILD_DEMOS)
	set(RL_MATH_FOUND TRUE)
	set(TARGETS ${TARGETS} rlmath)
endif(BUILD_RL_MATH)

if(BUILD_RL_UTIL)
	add_subdirectory(src/rl/util)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlThreadsDemo)
		add_subdirectory(demos/rlTimerDemo)
	endif(BUILD_DEMOS)
	set(RL_UTIL_FOUND TRUE)
	set(TARGETS ${TARGETS} rlutil)
endif(BUILD_RL_UTIL)

if(BUILD_RL_XML)
	add_subdirectory(src/rl/xml)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlLoadXmlDemo)
	endif(BUILD_DEMOS)
	set(RL_XML_FOUND TRUE)
	set(TARGETS ${TARGETS} rlxml)
endif(BUILD_RL_XML)

if(BUILD_RL_KIN)
	add_subdirectory(src/rl/kin)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlJacobianDemo)
		add_subdirectory(demos/rlPumaDemo)
	endif(BUILD_DEMOS)
	if(BUILD_TESTS)
		add_subdirectory(tests/rlInverseKinematicsTest)
	endif(BUILD_TESTS)
	set(RL_KIN_FOUND TRUE)
	set(TARGETS ${TARGETS} rlkin)
endif(BUILD_RL_KIN)

if(BUILD_RL_MDL)
	add_subdirectory(src/rl/mdl)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlDynamics1Demo)
		add_subdirectory(demos/rlDynamics2Demo)
		add_subdirectory(demos/rlInversePositionDemo)
	endif(BUILD_DEMOS)
	if(BUILD_TESTS)
		add_subdirectory(tests/rlDynamicsTest)
	endif(BUILD_TESTS)
	set(RL_MDL_FOUND TRUE)
	set(TARGETS ${TARGETS} rlmdl)
endif(BUILD_RL_MDL)

if(BUILD_RL_HAL)
	add_subdirectory(src/rl/hal)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlAxisControllerDemo)
		add_subdirectory(demos/rlCameraDemo)
		add_subdirectory(demos/rlGripperDemo)
		add_subdirectory(demos/rlLaserDemo)
		add_subdirectory(demos/rlRangeSensorDemo)
		add_subdirectory(demos/rlSixAxisForceTorqueSensorDemo)
	endif(BUILD_DEMOS)
	if(BUILD_TESTS)
		add_subdirectory(tests/rlHalEndianTest)
	endif(BUILD_TESTS)
	set(RL_HAL_FOUND TRUE)
	set(TARGETS ${TARGETS} rlhal)
endif(BUILD_RL_HAL)

if(BUILD_RL_SG)
	add_subdirectory(src/rl/sg)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlCollisionDemo)
		add_subdirectory(demos/rlViewDemo)
	endif(BUILD_DEMOS)
	set(RL_SG_FOUND TRUE)
	set(TARGETS ${TARGETS} rlsg)
endif(BUILD_RL_SG)

if(BUILD_RL_KIN AND BUILD_RL_SG)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlCoachKin)
	endif(BUILD_DEMOS)
endif(BUILD_RL_KIN AND BUILD_RL_SG)

if(BUILD_RL_MDL AND BUILD_RL_SG)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlCoachMdl)
	endif(BUILD_DEMOS)
endif(BUILD_RL_MDL AND BUILD_RL_SG)

if(BUILD_RL_PLAN)
	add_subdirectory(src/rl/plan)
	if(BUILD_DEMOS)
		add_subdirectory(demos/rlPlanDemo)
		add_subdirectory(demos/rlPrmDemo)
	endif(BUILD_DEMOS)
	if(BUILD_TESTS)
		add_subdirectory(tests/rlPrmTest)
	endif(BUILD_TESTS)
	set(RL_PLAN_FOUND TRUE)
	set(TARGETS ${TARGETS} rlplan)
endif(BUILD_RL_PLAN)

add_subdirectory(examples)

if(BUILD_EXTRAS)
	add_subdirectory(extras/wrlview)
endif(BUILD_EXTRAS)

if(UNIX)
	install(FILES robotics-library.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT demos)
endif(UNIX)

find_package(Doxygen)
find_package(LATEX)

if(DOXYGEN_FOUND)
	option(BUILD_DOCUMENTATION "Build documentation" FALSE)
	configure_file(${rl_SOURCE_DIR}/Doxyfile.in ${rl_BINARY_DIR}/Doxyfile)
	if(BUILD_DOCUMENTATION)
		add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${rl_BINARY_DIR}/Doxyfile)
	else(BUILD_DOCUMENTATION)
		add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${rl_BINARY_DIR}/Doxyfile)
	endif(BUILD_DOCUMENTATION)
	file(MAKE_DIRECTORY ${rl_BINARY_DIR}/doc)
	install(DIRECTORY ${rl_BINARY_DIR}/doc/ DESTINATION ${CMAKE_INSTALL_DOCDIR} COMPONENT documentation)
endif(DOXYGEN_FOUND)

set(CPACK_COMPONENT_DEMOS_DEPENDS examples libraries)
set(CPACK_COMPONENT_DEMOS_DISPLAY_NAME "Demos")
set(CPACK_COMPONENT_DOCUMENTATION_DISPLAY_NAME "Documentation")
set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Examples")
set(CPACK_COMPONENT_EXTRAS_DISPLAY_NAME "Extras")
set(CPACK_COMPONENT_HEADERS_DEPENDS libraries)
set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "Headers")
set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
set(CPACK_COMPONENTS_ALL demos documentation examples extras headers libraries)
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.roboticslibrary.org/")
set(CPACK_DEBIAN_PACKAGE_NAME "librl")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(
	CPACK_NSIS_MENU_LINKS
	"http://www.roboticslibrary.org/" "RL"
	"https://github.com/roboticslibrary" "RL on GitHub"
	"https://launchpad.net/roblib" "RL on Launchpad"
	"http://sourceforge.net/projects/roblib" "RL on SourceForge"
	"http://www.youtube.com/roboticslibrary" "RL on YouTube"
)
set(CPACK_NSIS_PACKAGE_NAME "Robotics Library ${VERSION}")
set(CPACK_NSIS_URL_INFO_ABOUT ${CPACK_DEBIAN_PACKAGE_HOMEPAGE})
set(CPACK_PACKAGE_CONTACT "Robotics Library Team <roblib-developers@lists.sourceforge.net>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Robotics Library is a self-contained C++ library for robot kinematics, visualization, motion planning, and control.")
set(CPACK_PACKAGE_DISPLAY_NAME "Robotics Library ${VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "rl-${VERSION}")
set(CPACK_PACKAGE_NAME "rl")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
set(CPACK_RPM_COMPONENT_INSTALL ON)
set(CPACK_RPM_demos_PACKAGE_REQUIRES "rl-examples")
set(CPACK_RPM_headers_PACKAGE_SUGGESTS "rl-documentation rl-libraries")
set(CPACK_RPM_libraries_PACKAGE_SUGGESTS "rl-demos rl-examples")
set(CPACK_RPM_PACKAGE_DESCRIPTION ${CPACK_PACKAGE_DESCRIPTION_SUMMARY})
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
set(CPACK_RPM_PACKAGE_LICENSE "BSD")
set(CPACK_RPM_PACKAGE_NAME "rl")
set(CPACK_RPM_PACKAGE_SUMMARY "Robotics Library")
set(CPACK_RPM_PACKAGE_URL ${CPACK_DEBIAN_PACKAGE_HOMEPAGE})

if(WIN32)
	if(CMAKE_CL_64)
		set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64") # http://public.kitware.com/Bug/view.php?id=14211
	endif()
endif()

string(REPLACE ";" "
" CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}")
string(REPLACE ";" "
" CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}")

include(CPack)

include(CMakePackageConfigHelpers)

set(RL_LIBRARIES ${TARGETS})
set(INTERFACE_INCLUDE_DIRECTORIES ${rl_SOURCE_DIR}/src)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/rl-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/rl-config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/rl
)

export(TARGETS ${TARGETS} FILE ${CMAKE_CURRENT_BINARY_DIR}/rl-export.cmake)

set(INTERFACE_INCLUDE_DIRECTORIES \${PACKAGE_PREFIX_DIR}/include)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/rl-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/rl-config-install.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/rl
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/rl-config-install.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/rl
	RENAME rl-config.cmake
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/rl-config-version.cmake
	VERSION ${VERSION}
	COMPATIBILITY ExactVersion
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/rl-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/rl
)

if(DEFINED TARGETS)
	install(
		EXPORT rl
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/rl
		FILE rl-export.cmake
	)
endif()
