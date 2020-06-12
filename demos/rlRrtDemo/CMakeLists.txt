find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ODE)
find_package(SOLID3)

if(Bullet_FOUND OR ODE_FOUND OR SOLID3_FOUND)
	add_executable(
		rlRrtDemo
		rlRrtDemo.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_include_directories(
		rlRrtDemo
		PUBLIC
		${Boost_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlRrtDemo
		mdl
		plan
		sg
	)
endif()
