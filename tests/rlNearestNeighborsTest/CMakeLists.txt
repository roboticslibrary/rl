find_package(Boost REQUIRED)

add_executable(
	rlNearestNeighborsTest
	iterator.h
	rlNearestNeighborsTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_include_directories(
	rlNearestNeighborsTest
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlNearestNeighborsTest
	math
)

add_test(
	NAME rlNearestNeighborsTest
	COMMAND rlNearestNeighborsTest
)
