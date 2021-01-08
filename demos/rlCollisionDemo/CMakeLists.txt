find_package(OpenGL REQUIRED)
find_package(SoQt)

if(RL_USE_QT5 AND SoQt5_FOUND)
	find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets QUIET)
endif()

if(Qt5_FOUND)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Widgets_LIBRARIES})
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

if(QT_FOUND AND SoQt_FOUND)
	if(BULLET_FOUND OR (CCD_FOUND AND FCL_FOUND) OR ODE_FOUND OR PQP_FOUND OR SOLID3_FOUND)
		set(
			HDRS
			BodyDelegate.h
			BodyModel.h
			MainWindow.h
			SoGradientBackground.h
		)
		
		set(
			SRCS
			BodyDelegate.cpp
			BodyModel.cpp
			MainWindow.cpp
			rlCollisionDemo.cpp
			SoGradientBackground.cpp
		)
		
		add_executable(
			rlCollisionDemo
			WIN32
			${HDRS}
			${SRCS}
			${rl_BINARY_DIR}/robotics-library.rc
		)
		
		target_compile_definitions(
			rlCollisionDemo
			PUBLIC
			${SoQt_DEFINITIONS}
		)
		
		target_include_directories(
			rlCollisionDemo
			PUBLIC
			${OPENGL_INCLUDE_DIR}
			${SoQt_INCLUDE_DIRS}
		)
		
		target_link_libraries(
			rlCollisionDemo
			sg
			${OPENGL_LIBRARIES}
			${QT_LIBRARIES}
			${SoQt_LIBRARIES}
		)
		
		set_target_properties(
			rlCollisionDemo
			PROPERTIES
			VERSION ${PROJECT_VERSION}
		)
		
		if(MSVC)
			set_target_properties(
				rlCollisionDemo
				PROPERTIES
				DEBUG_POSTFIX d
			)
		endif()
		
		install(
			TARGETS rlCollisionDemo
			COMPONENT demos
			ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
			RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
		
		if(MSVC AND BUILD_SHARED_LIBS)
			install(FILES $<TARGET_PDB_FILE:rlCollisionDemo> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT demos)
		endif()
		
		if(UNIX)
			configure_file(rlCollisionDemo.desktop.in rlCollisionDemo.desktop @ONLY)
			install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rlCollisionDemo.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT demos)
		endif()
		
		set(
			CPACK_NSIS_CREATE_ICONS_EXTRA
			${CPACK_NSIS_CREATE_ICONS_EXTRA}
			"CreateShortCut \\\\
				\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\rlCollisionDemo.lnk\\\" \\\\
				\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\rlCollisionDemo.exe\\\" \\\\
				\\\"\\\\
					\\\$\\\\\\\"$INSTDIR\\\\${CMAKE_INSTALL_DATADIR}\\\\rl-${PROJECT_VERSION}\\\\examples\\\\rlsg\\\\scene.xml\\\$\\\\\\\"\\\\
				\\\" \\\\
				\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
				\\\"Collision detection and distance computation demo with basic scene\\\""
			PARENT_SCOPE
		)
		set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\rlCollisionDemo.lnk\\\"" PARENT_SCOPE)
		
		set(
			WIX_SHORTCUTS
			${WIX_SHORTCUTS}
			"<Shortcut
				Arguments=\"&quot;[INSTALL_ROOT]${CMAKE_INSTALL_DATADIR}\\rl-${PROJECT_VERSION}\\examples\\rlsg\\scene.xml&quot;\"
				Description=\"Collision detection and distance computation demo with basic scene\"
				Id=\"CM_SP_bin.rlCollisionDemo.exe\"
				Name=\"rlCollisionDemo\"
				Target=\"[#CM_FP_bin.rlCollisionDemo.exe]\"
				WorkingDirectory=\"CM_DP_bin\"
			/>"
			PARENT_SCOPE
		)
	endif()
endif()
