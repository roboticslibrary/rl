find_package(Boost REQUIRED)

if(RL_BUILD_SG_BULLET OR RL_BUILD_SG_FCL OR RL_BUILD_SG_ODE OR RL_BUILD_SG_PQP OR RL_BUILD_SG_SOLID)
	add_executable(
		rlPrmTest
		rlPrmTest.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlPrmTest
		plan
		kin
		sg
		Boost::headers
	)
	
	if(RL_BUILD_SG_BULLET)
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
	
	if(RL_BUILD_SG_FCL)
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
	
	if(RL_BUILD_SG_ODE)
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
	
	if(RL_BUILD_SG_PQP)
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
	
	if(RL_BUILD_SG_SOLID)
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
