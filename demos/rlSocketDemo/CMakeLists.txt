find_package(Boost REQUIRED)

add_executable(
	rlSocketDemoClient
	rlSocketDemoClient.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlSocketDemoClient
	hal
	Boost::headers
)

add_executable(
	rlSocketDemoServer
	rlSocketDemoServer.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlSocketDemoServer
	hal
	Boost::headers
)
