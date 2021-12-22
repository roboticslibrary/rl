find_package(Boost REQUIRED)

add_executable(
	rlDynamics1Demo
	rlDynamics1Demo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlDynamics1Demo
	mdl
	Boost::headers
)

add_executable(
	rlDynamics1Planar2Demo
	rlDynamics1Planar2Demo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlDynamics1Planar2Demo
	mdl
	Boost::headers
)
