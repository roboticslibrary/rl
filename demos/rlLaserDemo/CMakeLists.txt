add_executable(
	rlLaserDemo
	rlLaserDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlLaserDemo
	hal
)
