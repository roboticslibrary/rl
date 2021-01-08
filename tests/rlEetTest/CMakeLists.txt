find_package(Boost REQUIRED)

find_package(Bullet)
find_package(fcl)
find_package(PQP)
find_package(solid3)

if(Bullet_FOUND OR fcl_FOUND OR PQP_FOUND OR solid3_FOUND)
	add_executable(
		rlEetTest
		rlEetTest.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_include_directories(
		rlEetTest
		PUBLIC
		${Boost_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlEetTest
		plan
		kin
		sg
	)
	
	if(Bullet_FOUND)
		add_test(
			NAME rlEetTestBulletBox6d300505Maze
			COMMAND rlEetTest
			bullet
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.mdl.xml
			${rl_SOURCE_DIR}/examples/rlmdl/box-6d-300505.sixDof.xml
			270 269
			2 1 1 0 0 0 1
			9 11 1 0 0 -0.70710678 0.70710678
		)
	endif()
	
	if(fcl_FOUND)
		add_test(
			NAME rlEetTestFclBox6d300505Maze
			COMMAND rlEetTest
			fcl
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.mdl.xml
			${rl_SOURCE_DIR}/examples/rlmdl/box-6d-300505.sixDof.xml
			242 241
			2 1 1 0 0 0 1
			9 11 1 0 0 -0.70710678 0.70710678
		)
	endif()
	
	if(PQP_FOUND)
		add_test(
			NAME rlEetTestPqpBox6d300505Maze
			COMMAND rlEetTest
			pqp
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.mdl.xml
			${rl_SOURCE_DIR}/examples/rlmdl/box-6d-300505.sixDof.xml
			239 238
			2 1 1 0 0 0 1
			9 11 1 0 0 -0.70710678 0.70710678
		)
	endif()
	
	if(solid3_FOUND)
		add_test(
			NAME rlEetTestSolidBox6d300505Maze
			COMMAND rlEetTest
			solid
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.mdl.xml
			${rl_SOURCE_DIR}/examples/rlmdl/box-6d-300505.sixDof.xml
			321 320
			2 1 1 0 0 0 1
			9 11 1 0 0 -0.70710678 0.70710678
		)
	endif()
endif()
