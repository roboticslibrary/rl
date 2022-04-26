find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

set(
	BASE_HDRS
	algorithm.h
	AlignedBox.h
	Array.h
	Circular.h
	CircularVector2.h
	CircularVector3.h
	CompositeFunction.h
	Constants.h
	Function.h
	GnatNearestNeighbors.h
	Kalman.h
	KdtreeBoundingBoxNearestNeighbors.h
	KdtreeNearestNeighbors.h
	LinearNearestNeighbors.h
	LowPass.h
	Matrix.h
	MatrixBaseAddons.h
	NestedFunction.h
	Pid.h
	Polynomial.h
	PolynomialQuaternion.h
	Quaternion.h
	QuaternionBaseAddons.h
	Real.h
	Rotation.h
	Spatial.h
	Spline.h
	SplineQuaternion.h
	Transform.h
	TransformAddons.h
	TrapezoidalVelocity.h
	TypeTraits.h
	Unit.h
	Units.h
	Vector.h
)
list(APPEND HDRS ${BASE_HDRS})

set(
	METRICS_HDRS
	metrics/L2.h
	metrics/L2Squared.h
)
list(APPEND HDRS ${METRICS_HDRS})

set(
	SPATIAL_HDRS
	spatial/ArticulatedBodyInertia.h
	spatial/ArticulatedBodyInertia.hxx
	spatial/ForceVector.h
	spatial/ForceVector.hxx
	spatial/MotionVector.h
	spatial/MotionVector.hxx
	spatial/PlueckerTransform.h
	spatial/PlueckerTransform.hxx
	spatial/RigidBodyInertia.h
	spatial/RigidBodyInertia.hxx
)
list(APPEND HDRS ${SPATIAL_HDRS})

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	add_library(math INTERFACE ${HDRS})
else()
	add_library(math INTERFACE)
	add_custom_target(math_sources SOURCES ${HDRS})
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
	target_compile_definitions(math INTERFACE EIGEN_DONT_ALIGN_STATICALLY)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(math INTERFACE cxx_std_11)
endif()

target_include_directories(
	math
	INTERFACE
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}>
)

target_link_libraries(
	math
	INTERFACE
	std
	Boost::headers
	Eigen3::Eigen
)

if(NOT CMAKE_VERSION VERSION_LESS 3.19)
	set_target_properties(
		math
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
endif()

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/math COMPONENT development)
install(FILES ${METRICS_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/math/metrics COMPONENT development)
install(FILES ${SPATIAL_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${PROJECT_VERSION}/rl/math/spatial COMPONENT development)

install(
	TARGETS math
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
