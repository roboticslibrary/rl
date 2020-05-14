find_package(Boost REQUIRED)

add_executable(
	rlSocketDemoClient
	rlSocketDemoClient.cpp
	${rl_BINARY_DIR}/robotics-library.rc
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
	${rl_BINARY_DIR}/robotics-library.rc
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
