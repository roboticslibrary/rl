project(rlPlanDemo)

find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

find_package(Bullet)
find_package(Ode)
find_package(Pqp)
find_package(Solid)
find_package(SoQt)

if(QT_FOUND AND SOQT_FOUND AND (BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND))
	set(
		HDRS
		ConfigurationDelegate.h
		ConfigurationModel.h
		ConfigurationSpaceScene.h
		ConfigurationSpaceThread.h
		MainWindow.h
		PlannerModel.h
		Thread.h
		Viewer.h
	)
	
	set(
		SRCS
		ConfigurationDelegate.cpp
		ConfigurationModel.cpp
		ConfigurationSpaceScene.cpp
		ConfigurationSpaceThread.cpp
		MainWindow.cpp
		PlannerModel.cpp
		rlPlanDemo.cpp
		Thread.cpp
		Viewer.cpp
		${rl_SOURCE_DIR}/robotics-library.rc
	)
	
	qt4_wrap_cpp(
		MOC_SRCS
		ConfigurationDelegate.h
		ConfigurationSpaceScene.h
		ConfigurationSpaceThread.h
		MainWindow.h
		Thread.h
		Viewer.h
		OPTIONS
		-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
	)
	
	add_executable(
		rlPlanDemo
		${HDRS}
		${SRCS}
		${MOC_SRCS}
	)
	
	target_compile_definitions(
		rlPlanDemo
		PUBLIC
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	
	target_include_directories(
		rlPlanDemo
		PUBLIC
		${OPENGL_INCLUDE_DIR}
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlPlanDemo
		rlplan
		rlkin
		rlsg
		${OPENGL_LIBRARIES}
		${QT_LIBRARIES}
		${SOQT_LIBRARIES}
	)
	
	set_target_properties(
		rlPlanDemo
		PROPERTIES
		VERSION ${VERSION}
		DEBUG_POSTFIX d
	)
	
	install(
		TARGETS rlPlanDemo
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(UNIX)
		install(FILES rlPlanDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif(UNIX)
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlPlanDemo.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlPlanDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlplan\\\\unimation-puma560_boxes_rrtConCon.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Path planning demo with Unimation Puma 560 and RRT algorithm\\\""
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlPlanDemo.lnk\\\"")
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlPlanDemo2.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlPlanDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlplan\\\\staeubli-tx60l_wall_rrtConCon.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Path planning demo with Staeubli TX-60L and RRT algorithm\\\""
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlPlanDemo2.lnk\\\"")
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlPlanDemo3.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlPlanDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlplan\\\\mitsubishi-rv6sl_boxes_rrtConCon.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Path planning demo with Mitsubishi RV-6SL and RRT algorithm\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlPlanDemo3.lnk\\\"" PARENT_SCOPE)
endif(QT_FOUND AND SOQT_FOUND AND (BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND))
