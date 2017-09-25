find_package(Bullet)
find_package(ccd)
find_package(Coin REQUIRED)
find_package(FCL)
find_package(ODE)
find_package(PQP)
find_package(SOLID3)

set(
	BASE_HDRS
	Body.h
	DepthScene.h
	DistanceScene.h
	Exception.h
	Model.h
	RaycastScene.h
	Scene.h
	Shape.h
	SimpleScene.h
)
list(APPEND HDRS ${BASE_HDRS})

set(
	BASE_SRCS
	Body.cpp
	DepthScene.cpp
	DistanceScene.cpp
	Exception.cpp
	Model.cpp
	RaycastScene.cpp
	Scene.cpp
	Shape.cpp
	SimpleScene.cpp
)
list(APPEND SRCS ${BASE_SRCS})

if(BULLET_FOUND)
	set(
		BULLET_HDRS
		bullet/Body.h
		bullet/Model.h
		bullet/Scene.h
		bullet/Shape.h
	)
	list(APPEND HDRS ${BULLET_HDRS})
	set(
		BULLET_SRCS
		bullet/Body.cpp
		bullet/Model.cpp
		bullet/Scene.cpp
		bullet/Shape.cpp
	)
	list(APPEND SRCS ${BULLET_SRCS})
endif()

if(Coin_FOUND)
	set(
		COIN_HDRS
		so/Body.h
		so/Model.h
		so/Scene.h
		so/Shape.h
	)
	list(APPEND HDRS ${COIN_HDRS})
	set(
		COIN_SRCS
		so/Body.cpp
		so/Model.cpp
		so/Scene.cpp
		so/Shape.cpp
	)
	list(APPEND SRCS ${COIN_SRCS})
endif()

if(CCD_FOUND AND FCL_FOUND)
	set(
		FCL_HDRS
		fcl/Body.h
		fcl/Model.h
		fcl/Scene.h
		fcl/Shape.h
	)
	list(APPEND HDRS ${FCL_HDRS})
	set(
		FCL_SRCS
		fcl/Body.cpp
		fcl/Model.cpp
		fcl/Scene.cpp
		fcl/Shape.cpp
	)
	list(APPEND SRCS ${FCL_SRCS})
endif()

if(ODE_FOUND)
	set(
		ODE_HDRS
		ode/Body.h
		ode/Model.h
		ode/Scene.h
		ode/Shape.h
	)
	list(APPEND HDRS ${ODE_HDRS})
	set(
		ODE_SRCS
		ode/Body.cpp
		ode/Model.cpp
		ode/Scene.cpp
		ode/Shape.cpp
	)
	list(APPEND SRCS ${ODE_SRCS})
endif()

if(PQP_FOUND)
	set(
		PQP_HDRS
		pqp/Body.h
		pqp/Model.h
		pqp/Scene.h
		pqp/Shape.h
	)
	list(APPEND HDRS ${PQP_HDRS})
	set(
		PQP_SRCS
		pqp/Body.cpp
		pqp/Model.cpp
		pqp/Scene.cpp
		pqp/Shape.cpp
	)
	list(APPEND SRCS ${PQP_SRCS})
endif()

if(SOLID3_FOUND)
	set(
		SOLID_HDRS
		solid/Body.h
		solid/Model.h
		solid/Scene.h
		solid/Shape.h
	)
	list(APPEND HDRS ${SOLID_HDRS})
	set(
		SOLID_SRCS
		solid/Body.cpp
		solid/Model.cpp
		solid/Scene.cpp
		solid/Shape.cpp
	)
	list(APPEND SRCS ${SOLID_SRCS})
endif()

add_library(
	sg
	${HDRS}
	${SRCS}
)

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(sg PUBLIC cxx_std_11)
endif()

target_link_libraries(
	sg
	math
	util
	xml
)

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg COMPONENT development)

target_include_directories(
	sg
	PUBLIC
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
)

if(BULLET_FOUND)
	target_compile_definitions(sg INTERFACE -DRL_SG_BULLET)
	target_include_directories(sg PUBLIC ${BULLET_INCLUDE_DIRS})
	target_link_libraries(sg ${BULLET_LIBRARIES})
	install(FILES ${BULLET_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/bullet COMPONENT development)
endif()

if(Coin_FOUND)
	target_compile_definitions(sg PUBLIC ${Coin_DEFINITIONS})
	target_include_directories(sg PUBLIC ${Coin_INCLUDE_DIRS})
	target_link_libraries(sg ${Coin_LIBRARIES})
	install(FILES ${COIN_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/so COMPONENT development)
endif()

if(CCD_FOUND AND FCL_FOUND)
	target_compile_definitions(sg INTERFACE -DRL_SG_FCL)
	target_compile_definitions(sg PUBLIC ${FCL_DEFINITIONS})
	target_include_directories(sg PUBLIC ${CCD_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS})
	target_link_libraries(sg ${FCL_LIBRARIES} ${CCD_LIBRARIES})
	install(FILES ${FCL_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/fcl COMPONENT development)
endif()

if(ODE_FOUND)
	target_compile_definitions(sg INTERFACE -DRL_SG_ODE)
	target_compile_definitions(sg PUBLIC ${ODE_DEFINITIONS})
	target_include_directories(sg PUBLIC ${ODE_INCLUDE_DIRS})
	target_link_libraries(sg ${ODE_LIBRARIES})
	install(FILES ${ODE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/ode COMPONENT development)
endif()

if(PQP_FOUND)
	target_compile_definitions(sg INTERFACE -DRL_SG_PQP)
	target_include_directories(sg PUBLIC ${PQP_INCLUDE_DIRS})
	target_link_libraries(sg ${PQP_LIBRARIES})
	install(FILES ${PQP_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/pqp COMPONENT development)
endif()

if(SOLID3_FOUND)
	target_compile_definitions(sg INTERFACE -DRL_SG_SOLID)
	target_include_directories(sg PUBLIC ${SOLID3_INCLUDE_DIRS})
	target_link_libraries(sg ${SOLID3_LIBRARIES})
	install(FILES ${SOLID_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/sg/solid COMPONENT development)
endif()

set_target_properties(
	sg
	PROPERTIES
	OUTPUT_NAME rlsg
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			sg
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			sg
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(
	TARGETS sg
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS sg
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:sg> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
