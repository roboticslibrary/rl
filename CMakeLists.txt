cmake_minimum_required(VERSION 2.8.11)

if(POLICY CMP0072)
	cmake_policy(SET CMP0072 NEW)
endif()

if(POLICY CMP0075)
	cmake_policy(SET CMP0075 NEW)
endif()

project(rl)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 7)
set(VERSION_PATCH 0)
set(VERSION_TWEAK 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CheckCXXCompilerFlag)
include(CMakeDependentOption)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include(GNUInstallDirs)
include(Qt4AutomocMocOptionsBoost)

set(Boost_ADDITIONAL_VERSIONS "1.72.0" "1.72" "1.71.0" "1.71" "1.70.0" "1.70" "1.69.0" "1.69" "1.68.0" "1.68" "1.67.0" "1.67" "1.66.0" "1.66" "1.65.1" "1.65.0" "1.65" "1.64.0" "1.64" "1.63.0" "1.63" "1.62.0" "1.62" "1.61.0" "1.61" "1.60.0" "1.60" "1.59.0" "1.59" "1.58.0" "1.58" "1.57.0" "1.57")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(NOT CMAKE_VERSION VERSION_LESS 3.1)
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
	check_cxx_compiler_flag("-std=c++0x" HAS_CXX_COMPILER_FLAG_STDCXX0X)
	if(HAS_CXX_COMPILER_FLAG_STDCXX0X)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	endif()
	check_cxx_compiler_flag("-std=c++11" HAS_CXX_COMPILER_FLAG_STDCXX11)
	if(HAS_CXX_COMPILER_FLAG_STDCXX11)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	endif()
endif()

if(WIN32)
	add_definitions(
		-D_ENABLE_EXTENDED_ALIGNED_STORAGE
		-D_USE_MATH_DEFINES
		-D_WIN32_WINNT=0x501
		-DNOMINMAX
		-DWIN32_LEAN_AND_MEAN
	)
endif()

if(WIN32)
	option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
else()
	option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()

option(RL_BUILD_DEMOS "Build demos" ON)
option(RL_BUILD_DOCUMENTATION "Build documentation" OFF)
option(RL_BUILD_EXTRAS "Build extras" ON)
option(RL_BUILD_MATH "Build mathematics component" ON)
option(RL_BUILD_TESTS "Build tests" ON)
option(RL_BUILD_UTIL "Build utility component" ON)
option(RL_BUILD_XML "Build XML abstraction layer component" ON)
option(RL_USE_QT5 "Prefer Qt5 over Qt4 if available" ON)

cmake_dependent_option(RL_BUILD_HAL "Build hardware abstraction layer component" ON "RL_BUILD_MATH;RL_BUILD_UTIL" OFF)
cmake_dependent_option(RL_BUILD_KIN "Build Denavit-Hartenberg kinematics component" ON "RL_BUILD_MATH;RL_BUILD_XML" OFF)
cmake_dependent_option(RL_BUILD_MDL "Build rigid body kinematics and dynamics component" ON "RL_BUILD_MATH;RL_BUILD_XML" OFF)
cmake_dependent_option(RL_BUILD_SG "Build scene graph abstraction component" ON "RL_BUILD_MATH;RL_BUILD_UTIL;RL_BUILD_XML" OFF)

cmake_dependent_option(RL_BUILD_PLAN "Build path planning component" ON "RL_BUILD_KIN;RL_BUILD_MATH;RL_BUILD_MDL;RL_BUILD_SG;RL_BUILD_UTIL;RL_BUILD_XML" OFF)

include_directories(BEFORE src/rl/std)

configure_file(robotics-library.rc.in robotics-library.rc)

add_subdirectory(src/rl/std)
add_subdirectory(src)
add_subdirectory(examples)

if(RL_BUILD_DEMOS)
	add_subdirectory(demos)
endif()

if(RL_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()

add_subdirectory(doc)

foreach(TARGET ${TARGETS})
	list(APPEND RL_LIBRARIES "rl::${TARGET}")
endforeach(TARGET)

set(RL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/rl/std)

configure_package_config_file(
	rl-config.cmake.in rl-config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rl-${VERSION}
)

export(
	TARGETS ${TARGETS}
	NAMESPACE rl::
	FILE ${CMAKE_CURRENT_BINARY_DIR}/rl-export.cmake
)

set(RL_INCLUDE_DIRS \${PACKAGE_PREFIX_DIR}/include/rl-${VERSION}/rl/std)

configure_package_config_file(
	rl-config.cmake.in rl-config-install.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rl-${VERSION}
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/rl-config-install.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rl-${VERSION}
	RENAME rl-config.cmake
	COMPONENT development
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/rl-config-version.cmake
	VERSION ${VERSION}
	COMPATIBILITY ExactVersion
)

install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/rl-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rl-${VERSION}
	COMPONENT development
)

if(DEFINED TARGETS)
	install(
		EXPORT rl
		DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rl-${VERSION}
		NAMESPACE rl::
		FILE rl-export.cmake
		COMPONENT development
	)
endif()

if(RL_BUILD_EXTRAS)
	add_subdirectory(extras)
endif()

set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT system)
include(InstallRequiredSystemLibraries)

include(CPackConfig)
include(CPack)
