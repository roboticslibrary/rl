find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ccd)
find_package(FCL)
find_package(PQP)
find_package(SOLID3)

if(BULLET_FOUND OR (CCD_FOUND AND FCL_FOUND) OR PQP_FOUND OR SOLID3_FOUND)
	add_executable(
		rlEetTest
		rlEetTest.cpp
	)
	
	target_include_directories(
		rlEetTest
		PUBLIC
		${Boost_INCLUDE_DIR}
	)
	
	target_link_libraries(
		rlEetTest
		plan
		kin
		sg
	)
	
	if(BULLET_FOUND)
		add_test(
			NAME rlEetTestBulletBox6d300505Maze
			COMMAND rlEetTest
			bullet
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.xml
			${rl_SOURCE_DIR}/examples/rlkin/box-6d-300505.xml
			270 269
			2 1 1 0 0 0
			9 11 1 0 0 -90
		)
	endif()
	
	if(CCD_FOUND AND FCL_FOUND)
		add_test(
			NAME rlEetTestFclBox6d300505Maze
			COMMAND rlEetTest
			fcl
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.xml
			${rl_SOURCE_DIR}/examples/rlkin/box-6d-300505.xml
			242 241
			2 1 1 0 0 0
			9 11 1 0 0 -90
		)
	endif()
	
	if(PQP_FOUND)
		add_test(
			NAME rlEetTestPqpBox6d300505Maze
			COMMAND rlEetTest
			pqp
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.xml
			${rl_SOURCE_DIR}/examples/rlkin/box-6d-300505.xml
			239 238
			2 1 1 0 0 0
			9 11 1 0 0 -90
		)
	endif()
	
	if(SOLID3_FOUND)
		add_test(
			NAME rlEetTestSolidBox6d300505Maze
			COMMAND rlEetTest
			solid
			${rl_SOURCE_DIR}/examples/rlsg/box-6d-300505_maze.xml
			${rl_SOURCE_DIR}/examples/rlkin/box-6d-300505.xml
			306 305
			2 1 1 0 0 0
			9 11 1 0 0 -90
		)
	endif()
endif()
