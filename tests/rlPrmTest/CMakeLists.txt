find_package(Boost REQUIRED)

find_package(Bullet)
find_package(fcl)
find_package(ODE)
find_package(PQP)
find_package(solid3)

if(Bullet_FOUND OR fcl_FOUND OR ODE_FOUND OR PQP_FOUND OR solid3_FOUND)
	add_executable(
		rlPrmTest
		rlPrmTest.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_include_directories(
		rlPrmTest
		PUBLIC
		${Boost_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlPrmTest
		plan
		kin
		sg
	)
	
	if(Bullet_FOUND)
		add_test(
			NAME rlPrmTestBulletUnimationPuma560Boxes1
			COMMAND rlPrmTest
			bullet
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(fcl_FOUND)
		add_test(
			NAME rlPrmTestFclUnimationPuma560Boxes1
			COMMAND rlPrmTest
			fcl
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
	
	if(solid3_FOUND)
		add_test(
			NAME rlPrmTestSolidUnimationPuma560Boxes1
			COMMAND rlPrmTest
			solid
			${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
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
			${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
			17 16
			0 0 0 0 0 90
			90 -180 90 0 0 0
			-80 -140 180 30 0 0
		)
	endif()
endif()
