find_package(Boost REQUIRED)

find_package(Bullet)
find_package(ODE)
find_package(SOLID3)

if(Bullet_FOUND OR ODE_FOUND OR SOLID3_FOUND)
	add_executable(
		rlPrmDemo
		rlPrmDemo.cpp
	)
	
	target_include_directories(
		rlPrmDemo
		PUBLIC
		${Boost_INCLUDE_DIR}
	)
	
	target_link_libraries(
		rlPrmDemo
		plan
		kin
		sg
	)
endif()
