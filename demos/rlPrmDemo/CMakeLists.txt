project(rlPrmDemo)

find_package(Boost REQUIRED)

find_package(Solid)

if(SOLID_FOUND)
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
		rlplan
		rlkin
		rlsg
	)
endif(SOLID_FOUND)
