project(byu2wrl)

find_package(Coin)

if(Coin_FOUND)
	add_executable(
		byu2wrl
		byu2wrl.cpp
	)
	
	target_link_libraries(
		byu2wrl
		Coin::Coin
	)
endif()
