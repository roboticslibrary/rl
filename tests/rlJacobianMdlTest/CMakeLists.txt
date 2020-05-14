add_executable(
	rlJacobianMdlTest
	rlJacobianMdlTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlJacobianMdlTest
	mdl
)

add_test(
	NAME rlJacobianMdlTestMitsubishiRv6sl
	COMMAND rlJacobianMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/mitsubishi-rv6sl.xml
)

add_test(
	NAME rlJacobianMdlTestComauSmart5Nj422027
	COMMAND rlJacobianMdlTest
	${rl_SOURCE_DIR}/examples/rlmdl/comau-smart5-nj4-220-27.xml
)
