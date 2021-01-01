find_package(Threads REQUIRED)

add_executable(
	rlThreadsDemo
	rlThreadsDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlThreadsDemo
	Threads::Threads
)
