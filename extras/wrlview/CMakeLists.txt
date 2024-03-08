project(wrlview VERSION 0.2.2)

if(NOT PROJECT_VERSION_TWEAK)
	set(PROJECT_VERSION_TWEAK 0)
endif()

include(CMakePushCheckState)

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

find_package(Coin)
find_package(OpenGL)

if(Coin_FOUND AND OPENGL_FOUND AND QT_FOUND AND SoQt_FOUND)
	configure_file(wrlview.rc.in wrlview.rc)
	
	add_executable(
		wrlview
		WIN32
		MainWindow.cpp
		MainWindow.h
		SoGradientBackground.cpp
		SoGradientBackground.h
		Widget.cpp
		Widget.h
		wrlview.cpp
		${CMAKE_CURRENT_BINARY_DIR}/wrlview.rc
	)
	
	cmake_push_check_state(RESET)
	if(CMAKE_VERSION VERSION_LESS 3.12)
		set(CMAKE_REQUIRED_INCLUDES ${Coin_INCLUDE_DIRS})
	endif()
	set(CMAKE_REQUIRED_LIBRARIES Coin::Coin)
	check_include_file_cxx(ForeignFiles/SoSTLFileKit.h HAVE_SOSTLFILEKIT_H)
	if(HAVE_SOSTLFILEKIT_H)
		target_compile_definitions(wrlview PRIVATE HAVE_SOSTLFILEKIT_H)
		check_cxx_source_compiles("
			#include <ForeignFiles/SoSTLFileKit.h>
			int main() { SoSTLFileKit* stlFileKit = new SoSTLFileKit(); stlFileKit->convert(); return 0; }
		" HAVE_SOSTLFILEKIT_CONVERT)
		if(HAVE_SOSTLFILEKIT_CONVERT)
			target_compile_definitions(wrlview PRIVATE HAVE_SOSTLFILEKIT_CONVERT)
		endif()
	endif()
	cmake_pop_check_state()
	
	target_link_libraries(
		wrlview
		Coin::Coin
		SoQt::SoQt
	)
	
	if(TARGET OpenGL::GL)
		target_link_libraries(wrlview OpenGL::GL)
	else()
		target_include_directories(wrlview PRIVATE ${OPENGL_INCLUDE_DIRS})
		target_link_libraries(wrlview ${OPENGL_LIBRARIES})
	endif()
	
	if(Qt6_FOUND)
		target_link_libraries(wrlview Qt6::Core Qt6::Gui Qt6::Network Qt6::OpenGL Qt6::Widgets)
	elseif(Qt5_FOUND)
		target_link_libraries(wrlview Qt5::Core Qt5::Gui Qt5::Network Qt5::OpenGL Qt5::Widgets)
	elseif(Qt4_FOUND)
		target_link_libraries(wrlview Qt4::QtCore Qt4::QtGui Qt4::QtNetwork Qt4::QtOpenGL)
	endif()
	
	set_target_properties(
		wrlview
		PROPERTIES
		VERSION ${PROJECT_VERSION}
	)
	
	if(MSVC)
		set_target_properties(
			wrlview
			PROPERTIES
			DEBUG_POSTFIX d
		)
	endif()
	
	install(
		TARGETS
		wrlview
		COMPONENT extras
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(UNIX)
		install(FILES wrlview.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT extras)
		install(FILES wrlview.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT extras)
	endif()
	
	set(
		CPACK_NSIS_CREATE_ICONS_EXTRA
		${CPACK_NSIS_CREATE_ICONS_EXTRA}
		"SetOutPath \\\"%HOMEDRIVE%%HOMEPATH%\\\"
		CreateShortCut \\\\
			\\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\wrlview.lnk\\\" \\\\
			\\\"$INSTDIR\\\\${CMAKE_INSTALL_BINDIR}\\\\wrlview.exe\\\" \\\\
			\\\"\\\" \\\\
			\\\"\\\" \\\"\\\" \\\"\\\" \\\"\\\" \\\\
			\\\"Viewer for VRML and Open Inventor files\\\""
		PARENT_SCOPE
	)
	set(CPACK_NSIS_DELETE_ICONS_EXTRA ${CPACK_NSIS_DELETE_ICONS_EXTRA} "Delete \\\"$SMPROGRAMS\\\\$START_MENU\\\\wrlview.lnk\\\"" PARENT_SCOPE)
	
	set(
		WIX_SHORTCUTS
		${WIX_SHORTCUTS}
		"<Shortcut
			Description=\"Viewer for VRML and Open Inventor files\"
			Id=\"CM_SP_bin.wrlview.exe\"
			Name=\"wrlview\"
			Target=\"[#CM_FP_bin.wrlview.exe]\"
			WorkingDirectory=\"CM_DP_bin\"
		/>"
		PARENT_SCOPE
	)
endif()
