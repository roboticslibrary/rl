project(tris2wrl)

find_package(Coin)

if(Coin_FOUND)
	add_executable(
		tris2wrl
		tris2wrl.cpp
	)
	
	target_compile_definitions(
		tris2wrl
		PUBLIC
		${Coin_DEFINITIONS}
	)
	
	target_include_directories(
		tris2wrl
		PUBLIC
		${Coin_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		tris2wrl
		${Coin_LIBRARIES}
	)
endif()
