add_executable(
	rlAxisControllerDemo
	rlAxisControllerDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlAxisControllerDemo
	hal
	math
)
