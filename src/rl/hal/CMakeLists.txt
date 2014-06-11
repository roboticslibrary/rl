project(rlhal)

find_package(Atidaq)
find_package(Comedi)
find_package(Libdc1394)

include(TestBigEndian)

test_big_endian(BIG_ENDIAN)

set(
	HDRS
	AxisController.h
	Camera.h
	CartesianPositionActuator.h
	CartesianPositionSensor.h
	Coach.h
	ComException.h
	Com.h
	DeviceException.h
	Device.h
	endian.h
	Exception.h
	ForceSensor.h
	Gnuplot.h
	Gripper.h
	JointPositionActuator.h
	JointPositionSensor.h
	JointTorqueActuator.h
	JointTorqueSensor.h
	JointVelocityActuator.h
	JointVelocitySensor.h
	LeuzeRs4.h
	Lidar.h
	MitsubishiH7Exception.h
	MitsubishiH7.h
	RangeSensor.h
	SchmersalLss300.h
	SchunkFpsF5.h
	Serial.h
	SickLms200.h
	SickS300.h
	SixAxisForceTorqueSensor.h
	Socket.h
	TcpSocket.h
	TimeoutException.h
	TorqueSensor.h
	types.h
	UdpSocket.h
	WeissWsg50.h
)
set(
	SRCS
	AxisController.cpp
	Camera.cpp
	CartesianPositionActuator.cpp
	CartesianPositionSensor.cpp
	Coach.cpp
	Com.cpp
	ComException.cpp
	Device.cpp
	DeviceException.cpp
	endian.cpp
	Exception.cpp
	ForceSensor.cpp
	Gnuplot.cpp
	Gripper.cpp
	JointPositionActuator.cpp
	JointPositionSensor.cpp
	JointTorqueActuator.cpp
	JointTorqueSensor.cpp
	JointVelocityActuator.cpp
	JointVelocitySensor.cpp
	LeuzeRs4.cpp
	Lidar.cpp
	MitsubishiH7.cpp
	MitsubishiH7Exception.cpp
	RangeSensor.cpp
	SchmersalLss300.cpp
	SchunkFpsF5.cpp
	Serial.cpp
	SickLms200.cpp
	SickS300.cpp
	SixAxisForceTorqueSensor.cpp
	Socket.cpp
	TcpSocket.cpp
	TimeoutException.cpp
	TorqueSensor.cpp
	UdpSocket.cpp
	WeissWsg50.cpp
)

if(ATIDAQ_FOUND AND COMEDI_FOUND)
	set(HDRS ${HDRS} Ati.h)
	set(SRCS ${SRCS} Ati.cpp)
endif(ATIDAQ_FOUND AND COMEDI_FOUND)

if(COMEDI_FOUND)
	set(HDRS ${HDRS} Comedi.h)
	set(HDRS ${HDRS} Jr3.h)
	set(SRCS ${SRCS} Comedi.cpp)
	set(SRCS ${SRCS} Jr3.cpp)
endif(COMEDI_FOUND)

if(LIBDC1394_FOUND)
	set(HDRS ${HDRS} Dc1394Camera.h)
	set(HDRS ${HDRS} Dc1394CameraException.h)
	set(SRCS ${SRCS} Dc1394Camera.cpp)
	set(SRCS ${SRCS} Dc1394CameraException.cpp)
endif(LIBDC1394_FOUND)

if(WIN32)
	add_library(
		rlhal
		STATIC
		${HDRS}
		${SRCS}
	)
else(WIN32)
	add_library(
		rlhal
		SHARED
		${HDRS}
		${SRCS}
	)
endif(WIN32)

if(BIG_ENDIAN)
	target_compile_definitions(rlhal PRIVATE -DHAVE_BIG_ENDIAN)
else(BIG_ENDIAN)
	target_compile_definitions(rlhal PRIVATE -DHAVE_LITTLE_ENDIAN)
endif(BIG_ENDIAN)

target_link_libraries(
	rlhal
	rlmath
	rlutil
)

if(ATIDAQ_FOUND AND COMEDI_FOUND)
	target_include_directories(rlhal PUBLIC ${ATIDAQ_INCLUDE_DIRS})
	target_link_libraries(rlhal ${ATIDAQ_LIBRARIES})
endif(ATIDAQ_FOUND AND COMEDI_FOUND)

if(COMEDI_FOUND)
	target_include_directories(rlhal PUBLIC ${COMEDI_INCLUDE_DIRS})
	target_link_libraries(rlhal ${COMEDI_LIBRARIES})
endif(COMEDI_FOUND)

if(LIBDC1394_FOUND)
	target_compile_definitions(rlhal PUBLIC ${LIBDC1394_DEFINITIONS})
	target_include_directories(rlhal PUBLIC ${LIBDC1394_INCLUDE_DIRS})
	target_link_libraries(rlhal ${LIBDC1394_LIBRARIES})
endif(LIBDC1394_FOUND)

if(QNXNTO)
	target_link_libraries(rlhal socket)
endif(QNXNTO)

if(WIN32)
	target_link_libraries(rlhal ws2_32)
endif(WIN32)

set_target_properties(
	rlhal
	PROPERTIES
	VERSION ${VERSION}
	DEBUG_POSTFIX d
)

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl/hal COMPONENT headers)

install(
	TARGETS	rlhal
	EXPORT rl
	COMPONENT libraries
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
