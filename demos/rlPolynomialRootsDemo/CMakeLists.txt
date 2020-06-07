find_package(Boost REQUIRED)

add_executable(
	rlPolynomialRootsDemo
	rlPolynomialRootsDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_include_directories(
	rlPolynomialRootsDemo
	PUBLIC
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(
	rlPolynomialRootsDemo
	math
)
