find_package(Boost REQUIRED)
find_package(NLopt)

set(
	HDRS
	Body.h
	Compound.h
	Cylindrical.h
	Dynamic.h
	Element.h
	Exception.h
	Factory.h
	Fixed.h
	Frame.h
	Helical.h
	InverseKinematics.h
	JacobianInverseKinematics.h
	Joint.h
	Kinematic.h
	Metric.h
	Model.h
	Prismatic.h
	Revolute.h
	Spherical.h
	Transform.h
	UrdfFactory.h
	World.h
	XmlFactory.h
)

set(
	SRCS
	Body.cpp
	Compound.cpp
	Cylindrical.cpp
	Dynamic.cpp
	Element.cpp
	Exception.cpp
	Fixed.cpp
	Factory.cpp
	Frame.cpp
	Helical.cpp
	InverseKinematics.cpp
	JacobianInverseKinematics.cpp
	Joint.cpp
	Kinematic.cpp
	Metric.cpp
	Model.cpp
	Prismatic.cpp
	Revolute.cpp
	Spherical.cpp
	Transform.cpp
	UrdfFactory.cpp
	World.cpp
	XmlFactory.cpp
)

if(NLOPT_FOUND)
	list(APPEND HDRS NloptInverseKinematics.h)
	list(APPEND SRCS NloptInverseKinematics.cpp)
endif()

add_library(
	mdl
	${HDRS}
	${SRCS}
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		mdl
		PUBLIC
		-DBOOST_ALL_NO_LIB
		-DBOOST_CHRONO_HEADER_ONLY
		-DBOOST_ERROR_CODE_HEADER_ONLY
		-DBOOST_SYSTEM_NO_DEPRECATED
	)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(mdl PUBLIC cxx_std_11)
endif()

target_include_directories(
	mdl
	PUBLIC
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	mdl
	math
	xml
)

if(NLOPT_FOUND)
	target_compile_definitions(mdl INTERFACE -DRL_MDL_NLOPT)
	target_include_directories(mdl PUBLIC ${NLOPT_INCLUDE_DIRS})
	target_link_libraries(mdl ${NLOPT_LIBRARY})
endif()

set_target_properties(
	mdl
	PROPERTIES
	OUTPUT_NAME rlmdl
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			mdl
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			mdl
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/mdl COMPONENT development)

install(
	TARGETS mdl
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS mdl
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:mdl> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
