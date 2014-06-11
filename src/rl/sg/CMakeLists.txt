project(rlsg)

find_package(Boost REQUIRED)

find_package(Bullet)
find_package(Coin REQUIRED)
find_package(Ode)
find_package(Pqp)
find_package(Solid)

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
set(HDRS ${BASE_HDRS})

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
set(SRCS ${BASE_SRCS})

if(BULLET_FOUND)
	set(
		BULLET_HDRS
		bullet/Body.h
		bullet/Model.h
		bullet/Scene.h
		bullet/Shape.h
	)
	set(HDRS ${HDRS} ${BULLET_HDRS})
	set(
		BULLET_SRCS
		bullet/Body.cpp
		bullet/Model.cpp
		bullet/Scene.cpp
		bullet/Shape.cpp
	)
	set(SRCS ${SRCS} ${BULLET_SRCS})
endif(BULLET_FOUND)

if(COIN_FOUND)
	set(
		COIN_HDRS
		so/Body.h
		so/Model.h
		so/Scene.h
		so/Shape.h
	)
	set(HDRS ${HDRS} ${COIN_HDRS})
	set(
		COIN_SRCS
		so/Body.cpp
		so/Model.cpp
		so/Scene.cpp
		so/Shape.cpp
	)
	set(SRCS ${SRCS} ${COIN_SRCS})
endif(COIN_FOUND)

if(ODE_FOUND)
	set(
		ODE_HDRS
		ode/Body.h
		ode/Model.h
		ode/Scene.h
		ode/Shape.h
	)
	set(HDRS ${HDRS} ${ODE_HDRS})
	set(
		ODE_SRCS
		ode/Body.cpp
		ode/Model.cpp
		ode/Scene.cpp
		ode/Shape.cpp
	)
	set(SRCS ${SRCS} ${ODE_SRCS})
endif(ODE_FOUND)

if(PQP_FOUND)
	set(
		PQP_HDRS
		pqp/Body.h
		pqp/Model.h
		pqp/Scene.h
		pqp/Shape.h
	)
	set(HDRS ${HDRS} ${PQP_HDRS})
	set(
		PQP_SRCS
		pqp/Body.cpp
		pqp/Model.cpp
		pqp/Scene.cpp
		pqp/Shape.cpp
	)
	set(SRCS ${SRCS} ${PQP_SRCS})
endif(PQP_FOUND)

if(SOLID_FOUND)
	set(
		SOLID_HDRS
		solid/Body.h
		solid/Model.h
		solid/Scene.h
		solid/Shape.h
	)
	set(HDRS ${HDRS} ${SOLID_HDRS})
	set(
		SOLID_SRCS
		solid/Body.cpp
		solid/Model.cpp
		solid/Scene.cpp
		solid/Shape.cpp
	)
	set(SRCS ${SRCS} ${SOLID_SRCS})
endif(SOLID_FOUND)

if(WIN32)
	add_library(
		rlsg
		STATIC
		${HDRS}
		${SRCS}
	)
else(WIN32)
	add_library(
		rlsg
		SHARED
		${HDRS}
		${SRCS}
	)
endif(WIN32)

target_include_directories(
	rlsg
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlsg
	rlmath
	rlutil
	rlxml
)

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg COMPONENT headers)

if(BULLET_FOUND)
	target_compile_definitions(rlsg PUBLIC -DRL_SG_HAVE_BULLET)
	target_include_directories(rlsg PUBLIC ${BULLET_INCLUDE_DIRS})
	target_link_libraries(rlsg ${BULLET_LIBRARIES})
	install(FILES ${BULLET_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg/bullet COMPONENT headers)
endif(BULLET_FOUND)

if(COIN_FOUND)
	target_compile_definitions(rlsg PUBLIC ${COIN_DEFINITIONS})
	target_include_directories(rlsg PUBLIC ${COIN_INCLUDE_DIRS})
	target_link_libraries(rlsg ${COIN_LIBRARIES})
	install(FILES ${COIN_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg/so COMPONENT headers)
endif(COIN_FOUND)

if(ODE_FOUND)
	target_compile_definitions(rlsg PUBLIC -DRL_SG_HAVE_ODE)
	target_compile_definitions(rlsg PUBLIC ${ODE_DEFINITIONS})
	target_include_directories(rlsg PUBLIC ${ODE_INCLUDE_DIRS})
	target_link_libraries(rlsg ${ODE_LIBRARIES})
	install(FILES ${ODE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg/ode COMPONENT headers)
endif(ODE_FOUND)

if(PQP_FOUND)
	target_compile_definitions(rlsg PUBLIC -DRL_SG_HAVE_PQP)
	target_include_directories(rlsg PUBLIC ${PQP_INCLUDE_DIRS})
	target_link_libraries(rlsg ${PQP_LIBRARIES})
	install(FILES ${PQP_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg/pqp COMPONENT headers)
endif(PQP_FOUND)

if(SOLID_FOUND)
	target_compile_definitions(rlsg PUBLIC -DRL_SG_HAVE_SOLID)
	target_include_directories(rlsg PUBLIC ${SOLID_INCLUDE_DIRS})
	target_link_libraries(rlsg ${SOLID_LIBRARIES})
	install(FILES ${SOLID_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/sg/solid COMPONENT headers)
endif(SOLID_FOUND)

set_target_properties(
	rlsg
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(
	TARGETS rlsg
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
