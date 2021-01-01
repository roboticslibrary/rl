find_package(Boost REQUIRED)

add_executable(
	rlJacobianDemo
	rlJacobianDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlJacobianDemo
	mdl
	Boost::headers
)
