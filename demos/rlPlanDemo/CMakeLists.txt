find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED)
find_package(SoQt)

if(RL_USE_QT5 AND SoQt5_FOUND)
	find_package(Qt5 COMPONENTS Core Gui OpenGL PrintSupport Widgets QUIET)
endif()

if(Qt5_FOUND)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5PrintSupport_LIBRARIES} ${Qt5Widgets_LIBRARIES})
	set(QT_FOUND Qt5_FOUND)
else()
	set(QT_USE_IMPORTED_TARGETS ON)
	find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
	set(QT_USE_QTMAIN ON)
	include(${QT_USE_FILE})
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Bullet)
find_package(ccd)
find_package(FCL)
find_package(ODE)
find_package(PQP)
find_package(SOLID3)

if(QT_FOUND AND SoQt_FOUND AND (Bullet_FOUND OR (CCD_FOUND AND FCL_FOUND) OR ODE_FOUND OR PQP_FOUND OR SOLID3_FOUND))
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
	
	target_compile_definitions(
		rlPlanDemo
		PUBLIC
		${SoQt_DEFINITIONS}
	)
	
	target_include_directories(
		rlPlanDemo
		PUBLIC
		${Boost_INCLUDE_DIR}
		${OPENGL_INCLUDE_DIR}
		${SoQt_INCLUDE_DIRS}
	)
	
	target_link_libraries(
		rlPlanDemo
		plan
		kin
		sg
		${OPENGL_LIBRARIES}
		${QT_LIBRARIES}
		${SoQt_LIBRARIES}
	)
	
	set_target_properties(
		rlPlanDemo
		PROPERTIES
		VERSION ${VERSION}
	)
	
	if(WIN32)
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
	
	if(MSVC AND BUILD_SHARED_LIBS AND NOT CMAKE_VERSION VERSION_LESS 3.1)
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
				\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${VERSION}\\\\examples\\\\rlplan\\\\unimation-puma560_boxes_rrtConCon.xml\\\$\\\\\\\"\\\\
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
			Arguments=\"&quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${VERSION}\\examples\\rlplan\\unimation-puma560_boxes_rrtConCon.xml&quot;\"
			Description=\"Path planning demo with Unimation Puma 560 and RRT algorithm\"
			Id=\"CM_SP_bin.rlPlanDemo.exe\"
			Name=\"rlPlanDemo\"
			Target=\"[#CM_FP_bin.rlPlanDemo.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
