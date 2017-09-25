find_package(Boost REQUIRED)

add_executable(
	rlPolynomialRootsDemo
	rlPolynomialRootsDemo.cpp
)

target_include_directories(
	rlPolynomialRootsDemo
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlPolynomialRootsDemo
	math
)
