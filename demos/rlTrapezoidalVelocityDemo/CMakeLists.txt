add_executable(
	rlTrapezoidalVelocityDemo
	rlTrapezoidalVelocityDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlTrapezoidalVelocityDemo
	math
)
