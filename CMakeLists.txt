cmake_minimum_required(VERSION 3.20)
project(vr_telepresence)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── quest_bridge_node ─────────────────────────────────────────────────────────
# Ontvangt TCP JSON van de Quest en publiceert QuaternionStamped
add_executable(quest_bridge_node src/quest_bridge_node.cpp)
target_link_libraries(quest_bridge_node PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS})

# ── arduino_serial ────────────────────────────────────────────────────────
# Zet QuaternionStamped om naar Euler en stuurt bytes naar de Arduino
add_executable(arduino_serial src/arduino_serial.cpp)
target_link_libraries(arduino_serial PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    Boost::system)

# ── arduino_parallel ────────────────────────────────────────────────────────
# Zet QuaternionStamped om naar lengtes van touwen en stuurt motor commands naar de Arduino
add_executable(arduino_parallel src/arduino_parallel.cpp)
target_link_libraries(arduino_parallel PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    ${Boost_LIBRARIES})

# ── emotion_input_node ────────────────────────────────────────────────────────
# Toetsenbordinput → publiceert emotie string naar Raspberry Pi
add_executable(emotion_input_node src/emotion_input_node.cpp)
target_link_libraries(emotion_input_node PUBLIC
    rclcpp::rclcpp
    ${std_msgs_TARGETS})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
    quest_bridge_node
    arduino_serial
    arduino_parallel
    emotion_input_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
