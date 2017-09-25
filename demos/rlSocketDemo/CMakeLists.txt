find_package(Boost REQUIRED)

add_executable(
	rlSocketDemoClient
	rlSocketDemoClient.cpp
)

target_include_directories(
	rlSocketDemoClient
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlSocketDemoClient
	hal
)

add_executable(
	rlSocketDemoServer
	rlSocketDemoServer.cpp
)

target_include_directories(
	rlSocketDemoServer
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlSocketDemoServer
	hal
)
