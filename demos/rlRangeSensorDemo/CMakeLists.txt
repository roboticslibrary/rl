add_executable(
	rlRangeSensorDemo
	rlRangeSensorDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlRangeSensorDemo
	hal
)
