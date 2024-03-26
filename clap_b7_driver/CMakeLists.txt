cmake_minimum_required(VERSION 3.8)
project(clap_b7_driver)


#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()


find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
        PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
        NAMES Geographic
)

include_directories(include)

set(msg_files
        "msg/ClapHeading.msg"
        "msg/ClapGpsPos.msg"
        "msg/ClapGpsVel.msg"
        "msg/ClapImu.msg"
        "msg/ClapIns.msg"
        "msg/ClapECEF.msg"
        "msg/ClapWheelOdom.msg"
        )


find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
        PATH_SUFFIXES GeographicLib
        )

rosidl_generate_interfaces(${PROJECT_NAME}
          ${msg_files}
          DEPENDENCIES
            std_msgs
        )

ament_export_dependencies(rosidl_default_runtime)


set(TARGET_SRC
        src/clap_binary_parser.cpp
        src/clap_b7_driver_node.cpp
        src/clap_b7_driver.cpp
        src/clap_msg_wrapper.cpp
        src/clap_config_params.cpp
        src/clap_publisher.cpp
        src/ll_to_utm_transform.cpp
        )

set(TARGET_SRC_CONFIG
        src/config_clap_b7_node.cpp
        src/config_clap_b7.cpp
        )

ament_auto_add_executable(clap_b7_driver_node
        ${TARGET_SRC}
        )

ament_auto_add_executable(config_clap_b7_node
        ${TARGET_SRC_CONFIG}
        )

target_link_libraries(clap_b7_driver_node
        Geographic
)


rosidl_target_interfaces(clap_b7_driver_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(config_clap_b7_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
