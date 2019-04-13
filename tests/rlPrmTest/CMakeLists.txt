find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ccd)
find_package(FCL)
find_package(ODE)
find_package(PQP)
find_package(SOLID3)

if(BULLET_FOUND OR (CCD_FOUND AND FCL_FOUND) OR ODE_FOUND OR PQP_FOUND OR SOLID3_FOUND)
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
		plan
		kin
		sg
	)
	
	if(BULLET_FOUND)
		add_test(
			NAME rlPrmTestBulletUnimationPuma560Boxes1
			COMMAND rlPrmTest
			bullet
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			NAME rlPrmTestBulletUnimationPuma560Boxes2
			COMMAND rlPrmTest
			bullet
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(CCD_FOUND AND FCL_FOUND)
		add_test(
			NAME rlPrmTestFclUnimationPuma560Boxes1
			COMMAND rlPrmTest
			fcl
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			NAME rlPrmTestFclUnimationPuma560Boxes2
			COMMAND rlPrmTest
			fcl
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(ODE_FOUND)
		add_test(
			NAME rlPrmTestOdeUnimationPuma560Boxes1
			COMMAND rlPrmTest
			ode
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			NAME rlPrmTestOdeUnimationPuma560Boxes2
			COMMAND rlPrmTest
			ode
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(PQP_FOUND)
		add_test(
			NAME rlPrmTestPqpUnimationPuma560Boxes1
			COMMAND rlPrmTest
			pqp
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			NAME rlPrmTestPqpUnimationPuma560Boxes2
			COMMAND rlPrmTest
			pqp
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(SOLID3_FOUND)
		add_test(
			NAME rlPrmTestSolidUnimationPuma560Boxes1
			COMMAND rlPrmTest
			solid
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			110 -200 60 0 0 0
			-20 0 90 -40 0 0
		)
		
		add_test(
			NAME rlPrmTestSolidUnimationPuma560Boxes2
			COMMAND rlPrmTest
			solid
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlkin/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
endif()
