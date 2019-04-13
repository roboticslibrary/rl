add_executable(
	rlHalEndianTest
	rlHalEndianTest.cpp
)

target_link_libraries(
	rlHalEndianTest
	hal
)

add_test(
	NAME rlHalEndianTest
	COMMAND rlHalEndianTest
)
