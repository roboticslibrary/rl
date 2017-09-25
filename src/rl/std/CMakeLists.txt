set(
	BASE_HDRS
	chrono
	condition_variable
	future
	iterator
	mutex
	random
	regex
	string
	thread
)
list(APPEND HDRS ${BASE_HDRS})

add_custom_target(std_sources SOURCES ${HDRS})

install(FILES ${BASE_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rl-${VERSION}/rl/std COMPONENT development)
