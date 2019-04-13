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
	NAME rlDynamicsTestMitsubishiRv6sl
	COMMAND rlDynamicsTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
	100
)

add_test(
	NAME rlDynamicsTestPlanar2
	COMMAND rlDynamicsTest
	${rl_SOURCE_DIR}/examples/rlmdl/planar2.xml
	100
)
