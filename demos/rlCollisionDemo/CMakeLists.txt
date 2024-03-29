find_package(OpenGL REQUIRED)
find_package(SoQt)

if(RL_USE_QT6 AND SoQt_HAVE_QT6)
	find_package(Qt6 COMPONENTS Core Gui OpenGL Widgets QUIET)
	set(QT_FOUND ${Qt6_FOUND})
endif()

if(NOT Qt6_FOUND AND RL_USE_QT5 AND SoQt_HAVE_QT5)
	find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets QUIET)
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
		BodyDelegate.h
		BodyModel.h
		MainWindow.h
		SoGradientBackground.h
		SoMaterialHighlightRenderAction.h
	)
	
	set(
		SRCS
		BodyDelegate.cpp
		BodyModel.cpp
		MainWindow.cpp
		rlCollisionDemo.cpp
		SoGradientBackground.cpp
		SoMaterialHighlightRenderAction.cpp
	)
	
	add_executable(
		rlCollisionDemo
		WIN32
		${HDRS}
		${SRCS}
		${rl_BINARY_DIR}/robotics-library.rc
	)
	
	target_link_libraries(
		rlCollisionDemo
		sg
		SoQt::SoQt
	)
	
	if(TARGET OpenGL::GL)
		target_link_libraries(rlCollisionDemo OpenGL::GL)
	else()
		target_include_directories(rlCollisionDemo PRIVATE ${OPENGL_INCLUDE_DIRS})
		target_link_libraries(rlCollisionDemo ${OPENGL_LIBRARIES})
	endif()
	
	if(Qt6_FOUND)
		target_link_libraries(rlCollisionDemo Qt6::Core Qt6::Gui Qt6::OpenGL Qt6::Widgets)
	elseif(Qt5_FOUND)
		target_link_libraries(rlCollisionDemo Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(rlCollisionDemo Qt4::QtCore Qt4::QtGui Qt4::QtOpenGL)
	endif()
	
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
