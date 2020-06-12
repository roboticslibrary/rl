find_package(Boost REQUIRED)

add_executable(
	rlJacobianDemo
	rlJacobianDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_include_directories(
	rlJacobianDemo
	PUBLIC
	${Boost_INCLUDE_DIRS}
)

target_link_libraries(
	rlJacobianDemo
	mdl
)
