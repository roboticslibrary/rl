project(csv2wrl)

find_package(Coin)
find_package(Eigen3)

if(Coin_FOUND AND Eigen3_FOUND)
	add_executable(
		csv2wrl
		csv2wrl.cpp
	)
	
	target_link_libraries(
		csv2wrl
		Coin::Coin
		Eigen3::Eigen
	)
endif()
