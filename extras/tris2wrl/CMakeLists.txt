project(tris2wrl)

find_package(Boost REQUIRED)
find_package(Coin)

if(Coin_FOUND)
	add_executable(
		tris2wrl
		tris2wrl.cpp
	)
	
	target_link_libraries(
		tris2wrl
		Boost::headers
		Coin::Coin
	)
endif()
