find_package(Boost REQUIRED)

set(
	HDRS
	Element.h
	Exception.h
	Frame.h
	Joint.h
	Kinematics.h
	Link.h
	Prismatic.h
	Puma.h
	Revolute.h
	Rhino.h
	Transform.h
)

set(
	SRCS
	Element.cpp
	Exception.cpp
	Frame.cpp
	Joint.cpp
	Kinematics.cpp
	Link.cpp
	Prismatic.cpp
	Puma.cpp
	Revolute.cpp
	Rhino.cpp
	Transform.cpp
)

add_library(
	kin
	${HDRS}
	${SRCS}
	${rl_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(kin PUBLIC cxx_std_11)
endif()

target_include_directories(
	kin
	PUBLIC
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(
	kin
	math
	std
	xml
)

set_target_properties(
	kin
	PROPERTIES
	OUTPUT_NAME rlkin
	POSITION_INDEPENDENT_CODE ON
	VERSION ${PROJECT_VERSION}
)

set_target_properties(
	kin
	PROPERTIES
	C_VISIBILITY_PRESET hidden
	CXX_VISIBILITY_PRESET hidden
	VISIBILITY_INLINES_HIDDEN ON
)

if(MSVC)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			kin
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			kin
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/kin COMPONENT development)

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS kin
		EXPORT rl
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
	install(
		TARGETS kin
		EXPORT rl
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
	if(BUILD_SHARED_LIBS)
		install(
			TARGETS kin
			EXPORT rl
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
		)
	endif()
endif()

if(MSVC AND BUILD_SHARED_LIBS)
	install(FILES $<TARGET_PDB_FILE:kin> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()

generate_export_header(
	kin
	EXPORT_FILE_NAME export.h
	PREFIX_NAME RL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/kin COMPONENT development)
