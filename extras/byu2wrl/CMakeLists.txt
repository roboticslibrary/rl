project(byu2wrl)

find_package(Coin)

if(Coin_FOUND)
	add_executable(
		byu2wrl
		byu2wrl.cpp
	)
	
	target_compile_definitions(
		byu2wrl
		PUBLIC
		${Coin_DEFINITIONS}
	)
	
	target_include_directories(
		byu2wrl
		PUBLIC
		${Coin_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		byu2wrl
		${Coin_LIBRARIES}
	)
endif()
