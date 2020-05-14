add_executable(
	rlInterpolatorDemo
	rlInterpolatorDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlInterpolatorDemo
	math
)
