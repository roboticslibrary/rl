project(rlCollisionDemo)

find_package(Boost REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

find_package(Bullet)
find_package(Ode)
find_package(Pqp)
find_package(Solid)
find_package(SoQt)

if(QT_FOUND AND SOQT_FOUND)
	if(BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND)
		set(
			HDRS
			MainWindow.h
			TestWidget.h
		)
		
		set(
			SRCS
			MainWindow.cpp
			rlCollisionDemo.cpp
			TestWidget.cpp
			${rl_SOURCE_DIR}/robotics-library.rc
		)
		
		qt4_wrap_cpp(
			MOC_SRCS
			TestWidget.h
			OPTIONS
			-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
		)
		
		add_executable(
			rlCollisionDemo
			${HDRS}
			${SRCS}
			${MOC_SRCS}
		)
		
		target_compile_definitions(
			rlCollisionDemo
			PUBLIC
			${QT_DEFINITIONS}
			${SOQT_DEFINITIONS}
		)
		
		target_include_directories(
			rlCollisionDemo
			PUBLIC
			${Boost_INCLUDE_DIR}
			${QT_INCLUDES}
			${SOQT_INCLUDE_DIRS}
		)
		
		target_link_libraries(
			rlCollisionDemo
			rlsg
			${QT_LIBRARIES}
			${SOQT_LIBRARIES}
		)
		
		set_target_properties(
			rlCollisionDemo
			PROPERTIES
			VERSION ${VERSION}
			DEBUG_POSTFIX d
		)
		
		install(
			TARGETS rlCollisionDemo
			COMPONENT demos
			ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
			RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
		
		if(UNIX)
			install(FILES rlCollisionDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
		endif(UNIX)
		
		set(
			CPACK_NSIS_EXTRA_INSTALL_COMMANDS
			${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
			"CreateShortCut \\\\
				\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlCollisionDemo.lnk\\\" \\\\
				\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlCollisionDemo.exe\\\" \\\\
				\\\"\\\\
					\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlsg\\\\scene.xml\\\$\\\\\\\"\\\\
				\\\" \\\\
				\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
				\\\"Collision detection and distance computation demo with basic scene\\\""
			PARENT_SCOPE
		)
		set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlCollisionDemo.lnk\\\"" PARENT_SCOPE)
	endif(BULLET_FOUND OR ODE_FOUND OR PQP_FOUND OR SOLID_FOUND)
endif(QT_FOUND AND SOQT_FOUND)
