project(rlCoachMdl)

find_package(Boost REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtOpenGL)
set(QT_USE_QTMAIN TRUE)
include(${QT_USE_FILE})

find_package(SoQt)

if(QT_FOUND AND SOQT_FOUND)
	set(
		HDRS
		ConfigurationDelegate.h
		ConfigurationModel.h
		MainWindow.h
		OperationalDelegate.h
		OperationalModel.h
		Server.h
		Socket.h
	)
	
	set(
		SRCS
		ConfigurationDelegate.cpp
		ConfigurationModel.cpp
		MainWindow.cpp
		OperationalDelegate.cpp
		OperationalModel.cpp
		rlCoachMdl.cpp
		Server.cpp
		Socket.cpp
		${rl_SOURCE_DIR}/robotics-library.rc
	)
	
	qt4_wrap_cpp(
		MOC_SRCS
		ConfigurationDelegate.h
		ConfigurationModel.h
		MainWindow.h
		OperationalDelegate.h
		OperationalModel.h
		Server.h
		Socket.h
		OPTIONS
		-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
	)
	
	add_executable(
		rlCoachMdl
		WIN32
		${HDRS}
		${SRCS}
		${MOC_SRCS}
	)
	
	target_compile_definitions(
		rlCoachMdl
		PUBLIC
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	
	target_include_directories(
		rlCoachMdl
		PUBLIC
		${Boost_INCLUDE_DIR}
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlCoachMdl
		rlmath
		rlmdl
		rlsg
		${QT_LIBRARIES}
		${SOQT_LIBRARIES}
	)
	
	set_target_properties(
		rlCoachMdl
		PROPERTIES
		VERSION ${VERSION}
		DEBUG_POSTFIX d
	)
	
	install(
		TARGETS rlCoachMdl
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(UNIX)
		install(FILES rlCoachMdl.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif(UNIX)
	
	set(
		CPACK_NSIS_EXTRA_INSTALL_COMMANDS
		${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlCoachMdl.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlCoachMdl.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlsg\\\\unimation-puma560_boxes.xml\\\$\\\\\\\" \\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl\\\\examples\\\\rlmdl\\\\unimation-puma560.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Visualization and kinematics demo with Unimation Puma 560\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlCoachMdl.lnk\\\"" PARENT_SCOPE)
endif(QT_FOUND AND SOQT_FOUND)
