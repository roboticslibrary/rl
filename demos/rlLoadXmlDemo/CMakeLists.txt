add_executable(
	rlLoadXmlDemo
	rlLoadXmlDemo.cpp
	${rl_BINARY_DIR}/robotics-library.rc
)

target_link_libraries(
	rlLoadXmlDemo
	xml
)
