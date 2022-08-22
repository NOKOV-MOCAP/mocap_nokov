add_executable(${PROJECT_NAME}_mocap_node
version.cpp
mocap_node.cpp
data_model.cpp
mocap_config.cpp
rigid_body_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_mocap_node 
${catkin_LIBRARIES}
${nokov_sdk_LIBRARIES})

set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                    OUTPUT_NAME mocap_node PREFIX "")

install(
TARGETS ${PROJECT_NAME}_mocap_node
DESTINATION lib/${PROJECT_NAME}
)
