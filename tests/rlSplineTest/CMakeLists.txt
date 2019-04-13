set(
	TESTS
	rlPolynomialExtremaTest
	rlPolynomialTest
	rlQuarticLinearQuarticTest
	rlSexticLinearSexticTest
	rlSplineScaleTest
	rlSplineTest
	rlSplineTranslationTest
	rlTrapezoidalAccelerationTest
)

foreach(TEST ${TESTS})
	add_executable(${TEST} ${TEST}.cpp)
	target_link_libraries(${TEST} math)
	add_test(NAME ${TEST} COMMAND ${TEST})
endforeach()
