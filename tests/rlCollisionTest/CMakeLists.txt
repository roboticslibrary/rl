find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ccd)
find_package(FCL)
find_package(ODE)
find_package(PQP)
find_package(SOLID3)

if(BULLET_FOUND OR (CCD_FOUND AND FCL_FOUND) OR ODE_FOUND OR PQP_FOUND OR SOLID3_FOUND)
	add_executable(
		rlCollisionTest
		rlCollisionTest.cpp
	)
	
	target_include_directories(
		rlCollisionTest
		PUBLIC
		${Boost_INCLUDE_DIR}
	)
	
	target_link_libraries(
		rlCollisionTest
		mdl
		sg
	)
	
	add_test(
		rlCollisionTestPuma560Boxes
		${CMAKE_CURRENT_BINARY_DIR}/rlCollisionTest
		${rl_SOURCE_DIR}/examples/rlsg/unimation-puma560_boxes.convex.xml
		${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
		0 0 0 0 0 0
	)
	
	add_executable(
		rlSceneCollisionTest
		rlSceneCollisionTest.cpp
	)
	
	target_link_libraries(
		rlSceneCollisionTest
		sg
	)
	
	add_test(
		rlSceneCollisionTestTwoTori1
		${CMAKE_CURRENT_BINARY_DIR}/rlSceneCollisionTest
		${CMAKE_CURRENT_SOURCE_DIR}/twotori.xml
	)
endif()
