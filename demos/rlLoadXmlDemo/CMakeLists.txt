project(rlLoadXmlDemo)

add_executable(
	rlLoadXmlDemo
	rlLoadXmlDemo.cpp
)

target_link_libraries(
	rlLoadXmlDemo
	rlxml
)
