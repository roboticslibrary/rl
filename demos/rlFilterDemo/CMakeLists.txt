add_executable(
	rlKalmanDemo
	rlKalmanDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlKalmanDemo
	math
)

add_executable(
	rlKalmanDemo2
	rlKalmanDemo2.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlKalmanDemo2
	math
)

add_executable(
	rlLowPassDemo
	rlLowPassDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlLowPassDemo
	math
)
