if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	find_package(Boost REQUIRED COMPONENTS thread)
else()
	find_package(Boost REQUIRED)
endif()

find_package(ATIDAQ)
find_package(cifX)
find_package(Comedi)
find_package(libdc1394)

cmake_dependent_option(RL_BUILD_HAL_ATIDAQ "Build ATIDAQ support" ON "RL_BUILD_HAL;ATIDAQ_FOUND;Comedi_FOUND" OFF)
cmake_dependent_option(RL_BUILD_HAL_CIFX "Build cifX support" ON "RL_BUILD_HAL;cifX_FOUND" OFF)
cmake_dependent_option(RL_BUILD_HAL_COMEDI "Build Comedi support" ON "RL_BUILD_HAL;Comedi_FOUND" OFF)
cmake_dependent_option(RL_BUILD_HAL_LIBDC1394 "Build libdc1394 support" ON "RL_BUILD_HAL;libdc1394_FOUND" OFF)

include(TestBigEndian)

test_big_endian(BIG_ENDIAN)

set(
	HDRS
	AnalogInput.h
	AnalogInputReader.h
	AnalogInputWriter.h
	AnalogOutput.h
	AnalogOutputReader.h
	AnalogOutputWriter.h
	AxisController.h
	Camera.h
	CartesianForceSensor.h
	CartesianPositionActuator.h
	CartesianPositionSensor.h
	CartesianVelocitySensor.h
	Coach.h
	ComException.h
	Com.h
	CyclicDevice.h
	DeviceException.h
	Device.h
	DigitalInput.h
	DigitalInputReader.h
	DigitalInputWriter.h
	DigitalOutput.h
	DigitalOutputReader.h
	DigitalOutputWriter.h
	Endian.h
	Exception.h
	Fieldbus.h
	ForceSensor.h
	Gnuplot.h
	Gripper.h
	JointAccelerationActuator.h
	JointCurrentSensor.h
	JointPositionActuator.h
	JointPositionSensor.h
	JointTorqueActuator.h
	JointTorqueSensor.h
	JointVelocityActuator.h
	JointVelocitySensor.h
	LeuzeRs4.h
	Lidar.h
	MitsubishiH7.h
	MitsubishiR3.h
	RangeSensor.h
	RobotiqModelC.h
	SchmersalLss300.h
	SchunkFpsF5.h
	Serial.h
	SickLms200.h
	SickS300.h
	SixAxisForceTorqueSensor.h
	Socket.h
	TimeoutException.h
	TorqueSensor.h
	UniversalRobotsDashboard.h
	UniversalRobotsRealtime.h
	UniversalRobotsRtde.h
	WeissException.h
	WeissKms40.h
	WeissWsg50.h
)

set(
	SRCS
	AnalogInput.cpp
	AnalogInputReader.cpp
	AnalogInputWriter.cpp
	AnalogOutput.cpp
	AnalogOutputReader.cpp
	AnalogOutputWriter.cpp
	AxisController.cpp
	Camera.cpp
	CartesianForceSensor.cpp
	CartesianPositionActuator.cpp
	CartesianPositionSensor.cpp
	CartesianVelocitySensor.cpp
	Coach.cpp
	Com.cpp
	ComException.cpp
	CyclicDevice.cpp
	Device.cpp
	DeviceException.cpp
	DigitalInput.cpp
	DigitalInputReader.cpp
	DigitalInputWriter.cpp
	DigitalOutput.cpp
	DigitalOutputReader.cpp
	DigitalOutputWriter.cpp
	Endian.cpp
	Exception.cpp
	Fieldbus.cpp
	ForceSensor.cpp
	Gnuplot.cpp
	Gripper.cpp
	JointAccelerationActuator.cpp
	JointCurrentSensor.cpp
	JointPositionActuator.cpp
	JointPositionSensor.cpp
	JointTorqueActuator.cpp
	JointTorqueSensor.cpp
	JointVelocityActuator.cpp
	JointVelocitySensor.cpp
	LeuzeRs4.cpp
	Lidar.cpp
	MitsubishiH7.cpp
	MitsubishiR3.cpp
	RangeSensor.cpp
	RobotiqModelC.cpp
	SchmersalLss300.cpp
	SchunkFpsF5.cpp
	Serial.cpp
	SickLms200.cpp
	SickS300.cpp
	SixAxisForceTorqueSensor.cpp
	Socket.cpp
	TimeoutException.cpp
	TorqueSensor.cpp
	UniversalRobotsDashboard.cpp
	UniversalRobotsRealtime.cpp
	UniversalRobotsRtde.cpp
	WeissException.cpp
	WeissKms40.cpp
	WeissWsg50.cpp
)

if(RL_BUILD_HAL_ATIDAQ)
	list(APPEND HDRS Ati.h)
	list(APPEND SRCS Ati.cpp)
endif()

if(RL_BUILD_HAL_CIFX)
	list(APPEND HDRS HilscherCifx.h)
	list(APPEND SRCS HilscherCifx.cpp)
endif()

if(RL_BUILD_HAL_COMEDI)
	list(APPEND HDRS Comedi.h)
	list(APPEND HDRS Jr3.h)
	list(APPEND SRCS Comedi.cpp)
	list(APPEND SRCS Jr3.cpp)
endif()

if(RL_BUILD_HAL_LIBDC1394)
	list(APPEND HDRS Dc1394Camera.h)
	list(APPEND SRCS Dc1394Camera.cpp)
endif()

add_library(
	hal
	${HDRS}
	${SRCS}
	${rl_BINARY_DIR}/robotics-library.rc
)

target_compile_definitions(
	hal
	PUBLIC
	BOOST_ALL_NO_LIB
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_compile_definitions(
		hal
		PUBLIC
		BOOST_ALL_NO_LIB
		BOOST_CHRONO_HEADER_ONLY
		BOOST_ERROR_CODE_HEADER_ONLY
		BOOST_SYSTEM_NO_DEPRECATED
	)
endif()

if(BIG_ENDIAN)
	target_compile_definitions(hal PRIVATE HAVE_BIG_ENDIAN)
else()
	target_compile_definitions(hal PRIVATE HAVE_LITTLE_ENDIAN)
endif()

if(NOT CMAKE_VERSION VERSION_LESS 3.8)
	target_compile_features(hal PUBLIC cxx_std_11)
endif()

target_link_libraries(
	hal
	math
	util
)

if(MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.00.23918)
	target_link_libraries(hal ${Boost_LIBRARIES})
endif()

target_include_directories(
	hal
	PUBLIC
	$<BUILD_INTERFACE:${rl_BINARY_DIR}/src>
	$<BUILD_INTERFACE:${rl_SOURCE_DIR}/src>
	$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}>
	${Boost_INCLUDE_DIR}
)

if(RL_BUILD_HAL_ATIDAQ)
	target_include_directories(hal PUBLIC ${ATIDAQ_INCLUDE_DIRS})
	target_link_libraries(hal ${ATIDAQ_LIBRARIES})
endif()

if(RL_BUILD_HAL_CIFX)
	target_include_directories(hal PUBLIC ${cifX_INCLUDE_DIRS})
	target_link_libraries(hal ${cifX_LIBRARIES})
endif()

if(RL_BUILD_HAL_COMEDI)
	target_include_directories(hal PUBLIC ${Comedi_INCLUDE_DIRS})
	target_link_libraries(hal ${Comedi_LIBRARIES})
endif()

if(RL_BUILD_HAL_LIBDC1394)
	target_compile_definitions(hal PUBLIC ${libdc1394_DEFINITIONS})
	target_include_directories(hal PUBLIC ${libdc1394_INCLUDE_DIRS})
	target_link_libraries(hal ${libdc1394_LIBRARIES})
endif()

if(QNXNTO)
	target_link_libraries(hal socket)
endif()

if(WIN32)
	target_link_libraries(hal ws2_32)
endif()

set_target_properties(
	hal
	PROPERTIES
	OUTPUT_NAME rlhal
	POSITION_INDEPENDENT_CODE ON
	VERSION ${VERSION}
)

if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
	set_target_properties(
		hal
		PROPERTIES
		C_VISIBILITY_PRESET hidden
		CXX_VISIBILITY_PRESET hidden
		VISIBILITY_INLINES_HIDDEN ON
	)
endif()

if(WIN32)
	if(BUILD_SHARED_LIBS)
		set_target_properties(
			hal
			PROPERTIES
			DEBUG_POSTFIX d
		)
	else()
		set_target_properties(
			hal
			PROPERTIES
			DEBUG_POSTFIX sd
			MINSIZEREL_POSTFIX s
			RELEASE_POSTFIX s
			RELWITHDEBINFO_POSTFIX s
		)
	endif()
endif()

install(FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/hal COMPONENT development)

install(
	TARGETS hal
	EXPORT rl
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)

if(BUILD_SHARED_LIBS)
	install(
		TARGETS hal
		EXPORT rl
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
	)
endif()

if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
	install(FILES $<TARGET_PDB_FILE:hal> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()

generate_export_header(
	hal
	EXPORT_FILE_NAME export.h
	PREFIX_NAME RL_
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/export.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/hal COMPONENT development)
