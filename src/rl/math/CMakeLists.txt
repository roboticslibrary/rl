project(rlmath)

find_package(Eigen REQUIRED)

set(
	HDRS
	ArticulatedBodyInertia.h
	ArticulatedBodyInertia.hxx
	Cubic.h
	ForceVector.h
	ForceVector.hxx
	Kalman.h
	Matrix.h
	MatrixBaseAddons.h
	MotionVector.h
	MotionVector.hxx
	Pid.h
	PlueckerTransform.h
	PlueckerTransform.hxx
	Quaternion.h
	QuaternionBaseAddons.h
	Quintic.h
	Real.h
	RigidBodyInertia.h
	RigidBodyInertia.hxx
	Rotation.h
	Spatial.h
	Transform.h
	TrapezoidalVelocity.h
	Unit.h
	Vector.h
)

add_library(
	rlmath
	${HDRS}
	dummy.cpp
)

target_include_directories(
	rlmath
	PUBLIC
	${EIGEN_INCLUDE_DIRS}
)

set_target_properties(
	rlmath
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/math COMPONENT headers)

install(
	TARGETS rlmath
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
