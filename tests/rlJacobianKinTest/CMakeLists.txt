add_executable(
	rlJacobianKinTest
	rlJacobianKinTest.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlJacobianKinTest
	kin
)

add_test(
	NAME rlJacobianKinTestMitsubishiRv6sl
	COMMAND rlJacobianKinTest
	${rl_SOURCE_DIR}/examples/rlkin/mitsubishi-rv6sl.xml
)

add_test(
	NAME rlJacobianKinTestStaeubliTx60l
	COMMAND rlJacobianKinTest
	${rl_SOURCE_DIR}/examples/rlkin/staeubli-tx60l.xml
)
