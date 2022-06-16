set(
	TESTS
	rlCompositeFunctionTest
	rlPolynomialAtRestTest
	rlPolynomialExtremaTest
	rlPolynomialTest
	rlQuarticLinearQuarticTest
	rlSexticLinearSexticTest
	rlSplineScaleTest
	rlSplineTest
	rlSplineTranslationTest
	rlTrapezoidalAccelerationTest
)

foreach(test IN LISTS TESTS)
	add_executable(
		${test}
		${test}.cpp
		${rl_BINARY_DIR}/robotics-library.rc
	)
	target_link_libraries(${test} math)
	add_test(NAME ${test} COMMAND ${test})
endforeach()
