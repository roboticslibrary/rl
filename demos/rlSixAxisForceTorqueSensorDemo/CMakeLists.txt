add_executable(
	rlSixAxisForceTorqueSensorDemo
	rlSixAxisForceTorqueSensorDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlSixAxisForceTorqueSensorDemo
	hal
	util
)
