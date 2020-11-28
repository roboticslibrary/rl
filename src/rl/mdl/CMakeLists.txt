find_package(Boost REQUIRED)
find_package(NLopt)

cmake_dependent_option(RL_BUILD_MDL_NLOPT "Build NLopt support" ON "RL_BUILD_MDL;NLOPT_FOUND" OFF)

set(
	HDRS
	AnalyticalInverseKinematics.h
	Body.h
	Compound.h
	Cylindrical.h
	Dynamic.h
	Element.h
	EulerCauchyIntegrator.h
	Exception.h
	Factory.h
	Fixed.h
	Frame.h
	Helical.h
	Integrator.h
	InverseKinematics.h
	IterativeInverseKinematics.h
	JacobianInverseKinematics.h
	Joint.h
	Kinematic.h
	Metric.h
	Model.h
	Prismatic.h
	Revolute.h
	RungeKuttaNystromIntegrator.h
	SixDof.h
	Spherical.h
	Transform.h
	UrdfFactory.h
	World.h
	XmlFactory.h
)

set(
	SRCS
	AnalyticalInverseKinematics.cpp
	Body.cpp
	Compound.cpp
	Cylindrical.cpp
	Dynamic.cpp
	Element.cpp
	EulerCauchyIntegrator.cpp
	Exception.cpp
	Fixed.cpp
	Factory.cpp
	Frame.cpp
	Helical.cpp
	Integrator.cpp
	InverseKinematics.cpp
	IterativeInverseKinematics.cpp
	JacobianInverseKinematics.cpp
	Joint.cpp
	Kinematic.cpp
	Metric.cpp
	Model.cpp
	Prismatic.cpp
	Revolute.cpp
	RungeKuttaNystromIntegrator.cpp
	SixDof.cpp
	Spherical.cpp
	Transform.cpp
	UrdfFactory.cpp
	World.cpp
	XmlFactory.cpp
)

if(RL_BUILD_MDL_NLOPT)
	list(APPEND HDRS NloptInverseKinematics.h)
	list(APPEND SRCS NloptInverseKinematics.cpp)
endif()

add_library(
	mdl
	${HDRS}
	${SRCS}
	${rl_BINARY_DIR}/robotics-library.rc
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(mdl PUBLIC cxx_std_11)
endif()

target_include_directories(
	mdl
	PUBLIC
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(
	mdl
	math
	std
	xml
)

if(RL_BUILD_MDL_NLOPT)
	target_compile_definitions(mdl INTERFACE RL_MDL_NLOPT)
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

if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
	set_target_properties(
		mdl
		PROPERTIES
		C_VISIBILITY_PRESET hidden
		CXX_VISIBILITY_PRESET hidden
		VISIBILITY_INLINES_HIDDEN ON
	)
endif()

if(MSVC)
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

if(NOT CMAKE_VERSION VERSION_LESS 3.12)
	install(
		TARGETS mdl
		EXPORT rl
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_COMPONENT development
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
	)
else()
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
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:mdl> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()

generate_export_header(
	mdl
	EXPORT_FILE_NAME export.h
	PREFIX_NAME RL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/mdl COMPONENT development)
