add_executable(
	rlHalEndianTest
	rlHalEndianTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlHalEndianTest
	hal
)

add_test(
	NAME rlHalEndianTest
	COMMAND rlHalEndianTest
)
