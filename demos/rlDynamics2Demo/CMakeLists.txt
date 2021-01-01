find_package(Boost REQUIRED)

add_executable(
	rlDynamics2Demo
	rlDynamics2Demo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlDynamics2Demo
	mdl
	Boost::headers
)
