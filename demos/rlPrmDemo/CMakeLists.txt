find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ODE)
find_package(solid3)

if(Bullet_FOUND OR ODE_FOUND OR solid3_FOUND)
	add_executable(
		rlPrmDemo
		rlPrmDemo.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_include_directories(
		rlPrmDemo
		PUBLIC
		${Boost_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlPrmDemo
		mdl
		plan
		sg
	)
endif()
