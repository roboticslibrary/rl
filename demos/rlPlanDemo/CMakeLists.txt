find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED)
find_package(SoQt)

if(RL_USE_QT6 AND SoQt_HAVE_QT6)
	find_package(Qt6 COMPONENTS Core Gui OpenGL PrintSupport Widgets QUIET)
	set(QT_FOUND ${Qt6_FOUND})
endif()

if(NOT Qt6_FOUND AND RL_USE_QT5 AND SoQt_HAVE_QT5)
	find_package(Qt5 COMPONENTS Core Gui OpenGL PrintSupport Widgets QUIET)
	set(QT_FOUND ${Qt5_FOUND})
endif()

if(NOT Qt6_FOUND AND NOT Qt5_FOUND)
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(QT_FOUND AND SoQt_FOUND AND (RL_BUILD_SG_BULLET OR RL_BUILD_SG_FCL OR RL_BUILD_SG_ODE OR RL_BUILD_SG_PQP OR RL_BUILD_SG_SOLID))
	set(
		HDRS
		ConfigurationDelegate.h
		ConfigurationModel.h
		ConfigurationSpaceModel.h
		ConfigurationSpaceScene.h
		ConfigurationSpaceThread.h
		GraphicsView.h
		MainWindow.h
		PlannerModel.h
		SoGradientBackground.h
		Thread.h
		Viewer.h
	)
	
	set(
		SRCS
		ConfigurationDelegate.cpp
		ConfigurationModel.cpp
		ConfigurationSpaceModel.cpp
		ConfigurationSpaceScene.cpp
		ConfigurationSpaceThread.cpp
		GraphicsView.cpp
		MainWindow.cpp
		PlannerModel.cpp
		rlPlanDemo.cpp
		SoGradientBackground.cpp
		Thread.cpp
		Viewer.cpp
	)
	
	add_executable(
		rlPlanDemo
		WIN32
		${HDRS}
		${SRCS}
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlPlanDemo
		plan
		kin
		sg
		Boost::headers
		SoQt::SoQt
	)
	
	if(TARGET OpenGL::GL)
		target_link_libraries(rlPlanDemo OpenGL::GL)
	else()
		target_include_directories(rlPlanDemo PRIVATE ${OPENGL_INCLUDE_DIRS})
		target_link_libraries(rlPlanDemo ${OPENGL_LIBRARIES})
	endif()
	
	if(Qt6_FOUND)
		target_link_libraries(rlPlanDemo Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::PrintSupport Qt6::Widgets)
	elseif(Qt5_FOUND)
		target_link_libraries(rlPlanDemo Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::PrintSupport Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(rlPlanDemo Qt4::QtCore Qt4::QtGui Qt4::QtOpenGL)
	endif()
	
	set_target_properties(
		rlPlanDemo
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			rlPlanDemo
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
	install(
		TARGETS rlPlanDemo
		COMPONENT demos
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(MSVC AND BUILD_SHARED_LIBS)
		install(FILES $<TARGET_PDB_FILE:rlPlanDemo> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT demos)
	endif()
	
	if(UNIX)
		configure_file(rlPlanDemo.desktop.in rlPlanDemo.desktop @ONLY)
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rlPlanDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlPlanDemo.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlPlanDemo.exe\\\" \\\\
			\\\"\\\\
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${PROJECT_VERSION}\\\\examples\\\\rlplan\\\\unimation-puma560_boxes_rrtConCon.mdl.xml\\\$\\\\\\\"\\\\
			\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Path planning demo with Unimation Puma 560 and RRT algorithm\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlPlanDemo.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Arguments=\"&quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${PROJECT_VERSION}\\examples\\rlplan\\unimation-puma560_boxes_rrtConCon.mdl.xml&quot;\"
			Description=\"Path planning demo with Unimation Puma 560 and RRT algorithm\"
			Id=\"CM_SP_bin.rlPlanDemo.exe\"
			Name=\"rlPlanDemo\"
			Target=\"[#CM_FP_bin.rlPlanDemo.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
