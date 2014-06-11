project(rlSixAxisForceTorqueSensorDemo)

if(COMEDI_FOUND)
	add_executable(
		rlSixAxisForceTorqueSensorDemo
		rlSixAxisForceTorqueSensorDemo.cpp
	)
	
	target_link_libraries(
		rlSixAxisForceTorqueSensorDemo
		rlhal
		rlutil
	)
endif(COMEDI_FOUND)
