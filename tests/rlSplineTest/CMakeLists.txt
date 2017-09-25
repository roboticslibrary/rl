add_executable(
	rlPolynomialTest
	rlPolynomialTest.cpp
)

target_link_libraries(
	rlPolynomialTest
	math
)

add_test(
	rlPolynomialTest
	${CMAKE_CURRENT_BINARY_DIR}/rlPolynomialTest
)

add_executable(
	rlQuarticLinearQuarticTest
	rlQuarticLinearQuarticTest.cpp
)

target_link_libraries(
	rlQuarticLinearQuarticTest
	math
)

add_test(
	rlQuarticLinearQuarticTest
	${CMAKE_CURRENT_BINARY_DIR}/rlQuarticLinearQuarticTest
)

add_executable(
	rlSexticLinearSexticTest
	rlSexticLinearSexticTest.cpp
)

target_link_libraries(
	rlSexticLinearSexticTest
	math
)

add_test(
	rlSexticLinearSexticTest
	${CMAKE_CURRENT_BINARY_DIR}/rlSexticLinearSexticTest
)

add_executable(
	rlSplineTest
	rlSplineTest.cpp
)

target_link_libraries(
	rlSplineTest
	math
)

add_test(
	rlSplineTestConstructCubicAndLinearQuartic
	${CMAKE_CURRENT_BINARY_DIR}/rlSplineTest
)

add_executable(
	rlTrapezoidalAccelerationTest
	rlTrapezoidalAccelerationTest.cpp
)

target_link_libraries(
	rlTrapezoidalAccelerationTest
	math
)

add_test(
	rlTrapezoidalAccelerationTest
	${CMAKE_CURRENT_BINARY_DIR}/rlTrapezoidalAccelerationTest
)

add_executable(
	rlSplineScaleTest
	rlSplineScaleTest.cpp
)

target_link_libraries(
	rlSplineScaleTest
	math
)

add_test(
	rlSplineScaleTestLinearQuartic
	${CMAKE_CURRENT_BINARY_DIR}/rlSplineScaleTest
)

add_executable(
	rlSplineTranslationTest
	rlSplineTranslationTest.cpp
)

target_link_libraries(
	rlSplineTranslationTest
	math
)

add_test(
	rlSplineTranslationTest
	${CMAKE_CURRENT_BINARY_DIR}/rlSplineTranslationTest
)


add_executable(
	rlPolynomialExtremaTest
	rlPolynomialExtremaTest.cpp
)

target_link_libraries(
	rlPolynomialExtremaTest
	math
)

add_test(
	rlPolynomialExtremaTest
	${CMAKE_CURRENT_BINARY_DIR}/rlPolynomialExtremaTest
)
