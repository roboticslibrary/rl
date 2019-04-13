add_executable(
	rlInverseKinematicsMdlTest
	rlInverseKinematicsMdlTest.cpp
)

target_link_libraries(
	rlInverseKinematicsMdlTest
	mdl
)

add_test(
	NAME rlInverseKinematicsMdlTestMitsubishiRv6sl
	COMMAND rlInverseKinematicsMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
)

add_test(
	NAME rlInverseKinematicsMdlTestUnimationPuma560
	COMMAND rlInverseKinematicsMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
)
