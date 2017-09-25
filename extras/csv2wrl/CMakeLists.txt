project(csv2wrl)

find_package(Coin)
find_package(Eigen3)

if(Coin_FOUND AND EIGEN3_FOUND)
	add_executable(
		csv2wrl
		csv2wrl.cpp
	)
	
	target_compile_definitions(
		csv2wrl
		PUBLIC
		${Coin_DEFINITIONS}
	)
	
	target_include_directories(
		csv2wrl
		PUBLIC
		${Coin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		csv2wrl
		${Coin_LIBRARIES}
	)
endif()
