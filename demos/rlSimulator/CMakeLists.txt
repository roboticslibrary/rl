find_package(OpenGL REQUIRED)
find_package(SoQt)

if(RL_USE_QT6 AND SoQt_HAVE_QT6)
	find_package(Qt6 COMPONENTS Core Gui Network OpenGL Widgets QUIET)
	set(QT_FOUND ${Qt6_FOUND})
endif()

if(NOT Qt6_FOUND AND RL_USE_QT5 AND SoQt_HAVE_QT5)
	find_package(Qt5 COMPONENTS Core Gui Network OpenGL Widgets QUIET)
	set(QT_FOUND ${Qt5_FOUND})
endif()

if(NOT Qt6_FOUND AND NOT Qt5_FOUND)
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtOpenGL)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(QT_FOUND AND SoQt_FOUND)
	set(
		HDRS
		ConfigurationDelegate.h
		ConfigurationModel.h
		MainWindow.h
		OperationalDelegate.h
		OperationalModel.h
		Server.h
		Socket.h
		SoGradientBackground.h
	)
	
	set(
		SRCS
		rlSimulator.cpp
		ConfigurationDelegate.cpp
		ConfigurationModel.cpp
		MainWindow.cpp
		OperationalDelegate.cpp
		OperationalModel.cpp
		Server.cpp
		Socket.cpp
		SoGradientBackground.cpp
	)
	
	add_executable(
		rlSimulator
		WIN32
		${HDRS}
		${SRCS}
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlSimulator
		math
		mdl
		sg
		SoQt::SoQt
	)
	
	if(TARGET OpenGL::GL)
		target_link_libraries(rlSimulator OpenGL::GL)
	else()
		target_include_directories(rlSimulator PRIVATE ${OPENGL_INCLUDE_DIRS})
		target_link_libraries(rlSimulator ${OPENGL_LIBRARIES})
	endif()
	
	if(Qt6_FOUND)
		target_link_libraries(rlSimulator Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Network Qt6::Widgets)
	elseif(Qt5_FOUND)
		target_link_libraries(rlSimulator Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Network Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(rlSimulator Qt4::QtCore Qt4::QtGui Qt4::QtNetwork Qt4::QtOpenGL)
	endif()
	
	set_target_properties(
		rlSimulator
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			rlSimulator
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
	install(
		TARGETS rlSimulator
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(MSVC AND BUILD_SHARED_LIBS)
		install(FILES $<TARGET_PDB_FILE:rlSimulator> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT demos)
	endif()
	
	if(UNIX)
		configure_file(rlSimulator.desktop.in rlSimulator.desktop @ONLY)
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rlSimulator.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlSimulator.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlSimulator.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${PROJECT_VERSION}\\\\examples\\\\rlsg\\\\unimation-puma560_boxes.xml\\\$\\\\\\\" \\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${PROJECT_VERSION}\\\\examples\\\\rlmdl\\\\unimation-puma560.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Simulation demo with Unimation Puma 560\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlSimulator.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Arguments=\"&quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${PROJECT_VERSION}\\examples\\rlsg\\unimation-puma560_boxes.xml&quot; &quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${PROJECT_VERSION}\\examples\\rlmdl\\unimation-puma560.xml&quot;\"
			Description=\"Simulation demo with Unimation Puma 560\"
			Id=\"CM_SP_bin.rlSimulator.exe\"
			Name=\"rlSimulator\"
			Target=\"[#CM_FP_bin.rlSimulator.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()

