find_package(Boost REQUIRED)

if(RL_BUILD_SG_BULLET OR RL_BUILD_SG_FCL OR RL_BUILD_SG_ODE OR RL_BUILD_SG_PQP OR RL_BUILD_SG_SOLID)
	add_executable(
		rlCollisionTest
		rlCollisionTest.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlCollisionTest
		mdl
		sg
		Boost::headers
	)
	
	add_test(
		NAME rlCollisionTestPuma560Boxes
		COMMAND rlCollisionTest
		${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
		${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
		0 0 0 0 0 0
	)
	
	add_executable(
		rlSceneCollisionTest
		rlSceneCollisionTest.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlSceneCollisionTest
		sg
	)
	
	add_test(
		NAME rlSceneCollisionTestTwoTori1
		COMMAND rlSceneCollisionTest
		${CMAKE_CURRENT_SOURCE_DIR}/twotori.xml
	)
endif()
