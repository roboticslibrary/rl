find_package(Boost REQUIRED)

add_executable(
	rlJacobianDemo
	rlJacobianDemo.cpp
)

target_include_directories(
	rlJacobianDemo
	PUBLIC
	${Boost_INCLUDE_DIR}
)

target_link_libraries(
	rlJacobianDemo
	kin
)
