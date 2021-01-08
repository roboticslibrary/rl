install(
	DIRECTORY rlkin
	DESTINATION ${CMAKE_INSTALL_DATADIR}/rl-${PROJECT_VERSION}/examples
	COMPONENT examples
)

install(
	DIRECTORY rlmdl
	DESTINATION ${CMAKE_INSTALL_DATADIR}/rl-${PROJECT_VERSION}/examples
	COMPONENT examples
)

install(
	DIRECTORY rlplan
	DESTINATION ${CMAKE_INSTALL_DATADIR}/rl-${PROJECT_VERSION}/examples
	COMPONENT examples
)

install(
	DIRECTORY rlsg
	DESTINATION ${CMAKE_INSTALL_DATADIR}/rl-${PROJECT_VERSION}/examples
	COMPONENT examples
)

install(FILES unimation-puma560.mdl.sg.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/rl-${PROJECT_VERSION}/examples COMPONENT examples)

if(UNIX)
	install(FILES vrml.lang DESTINATION ${CMAKE_INSTALL_DATADIR}/gtksourceview-3.0/language-specs COMPONENT examples)
endif()
