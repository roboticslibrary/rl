add_executable(
	rlQuaternionDemo
	rlQuaternionDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlQuaternionDemo
	math
)
