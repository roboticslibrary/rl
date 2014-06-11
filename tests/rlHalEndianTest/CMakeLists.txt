project(rlHalEndianTest)

add_executable(
	rlHalEndianTest
	rlHalEndianTest.cpp
)

target_link_libraries(
	rlHalEndianTest
	rlhal
)

add_test(
	rlHalEndianTest
	${CMAKE_CURRENT_BINARY_DIR}/rlHalEndianTest
)
