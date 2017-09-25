add_executable(
	rlHalEndianTest
	rlHalEndianTest.cpp
)

target_link_libraries(
	rlHalEndianTest
	hal
)

add_test(
	rlHalEndianTest
	${CMAKE_CURRENT_BINARY_DIR}/rlHalEndianTest
)
