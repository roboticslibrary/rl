add_executable(
	rlSpatialTest
	rlSpatialTest.cpp
)

target_link_libraries(
	rlSpatialTest
	math
)

add_test(
	rlSpatialTest
	${CMAKE_CURRENT_BINARY_DIR}/rlSpatialTest
)
