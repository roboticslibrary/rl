add_executable(
	rlInverseKinematicsMdlTest
	rlInverseKinematicsMdlTest.cpp
)

target_compile_definitions(
	rlInverseKinematicsMdlTest
	PRIVATE
	-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(
	rlInverseKinematicsMdlTest
	mdl
)

add_test(
	rlInverseKinematicsMdlTestMitsubishiRv6sl
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
)

add_test(
	rlInverseKinematicsMdlTestUnimationPuma560
	${CMAKE_CURRENT_BINARY_DIR}/rlInverseKinematicsMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/unimation-puma560.xml
)
