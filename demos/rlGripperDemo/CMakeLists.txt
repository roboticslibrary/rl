add_executable(
	rlGripperDemo
	rlGripperDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlGripperDemo
	hal
	util
)

add_executable(
	rlRobotiqModelCDemo
	rlRobotiqModelCDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlRobotiqModelCDemo
	hal
)
