add_executable(
	rlDynamicsTest
	rlDynamicsTest.cpp
)

target_link_libraries(
	rlDynamicsTest
	mdl
	util
)

add_test(
	rlDynamicsTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlDynamicsTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
	100
)

add_test(
	rlDynamicsTestPlanar2
	${CMAKE_CURRENT_BINARY_DIR}/rlDynamicsTest
	${rl_SOURCE_DIR}/examples/rlmdl/planar2.xml
	100
)
