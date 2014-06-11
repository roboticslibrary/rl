project(rlPrmTest)

find_package(Boost REQUIRED)

find_package(Bullet)
find_package(Ode)
find_package(Pqp)
find_package(Solid)

if(BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND)
	add_executable(
		rlPrmTest
		rlPrmTest.cpp
	)
	
	target_include_directories(
		rlPrmTest
		PUBLIC
		${Boost_INCLUDE_DIR}
	)
	
	target_link_libraries(
		rlPrmTest
		rlplan
		rlkin
		rlsg
	)
	
	if(BULLET_FOUND)
		add_test(
			rlPrmTestBulletUnimationPuma560Boxes1
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			bullet
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			rlPrmTestBulletUnimationPuma560Boxes2
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			bullet
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif(BULLET_FOUND)
	
	if(ODE_FOUND)
		add_test(
			rlPrmTestOdeUnimationPuma560Boxes1
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			ode
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			rlPrmTestOdeUnimationPuma560Boxes2
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			ode
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif(ODE_FOUND)
	
	if(PQP_FOUND)
		add_test(
			rlPrmTestPqpUnimationPuma560Boxes1
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			pqp
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			rlPrmTestPqpUnimationPuma560Boxes2
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			pqp
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif(PQP_FOUND)
	
	if(SOLID_FOUND)
		add_test(
			rlPrmTestSolidUnimationPuma560Boxes1
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			solid
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			rlPrmTestSolidUnimationPuma560Boxes2
			${CMAKE_CURRENT_BINARY_DIR}/rlPrmTest
			solid
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlsg/unimation-puma560_boxes.convex.xml
			${CMAKE_CURRENT_SOURCE_DIR}/../../examples/rlkin/unimation-puma560.xml
			17 16
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif(SOLID_FOUND)
endif(BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND)
