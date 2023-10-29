add_executable(
	rlCoordinatesPolarTest
	rlCoordinatesPolarTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlCoordinatesPolarTest
	math
)

add_test(
	NAME rlCoordinatesPolarTest
	COMMAND rlCoordinatesPolarTest
)

add_executable(
	rlCoordinatesSpatialTest
	rlCoordinatesSpatialTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlCoordinatesSpatialTest
	math
)

add_test(
	NAME rlCoordinatesSpatialTest
	COMMAND rlCoordinatesSpatialTest
)
