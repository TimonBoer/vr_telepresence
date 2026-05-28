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

# ── quest_bridge_node ─────────────────────────────────────────────────────────
# Ontvangt TCP JSON van de Quest en publiceert QuaternionStamped
add_executable(quest_bridge_node src/quest_bridge_node.cpp)
ament_target_dependencies(quest_bridge_node rclcpp geometry_msgs)

# ── arduino_servo_node ────────────────────────────────────────────────────────
# Zet QuaternionStamped om naar Euler en stuurt bytes naar de Arduino
add_executable(arduino_servo_node src/arduino_servo_node.cpp)
ament_target_dependencies(arduino_servo_node rclcpp geometry_msgs)
target_link_libraries(arduino_servo_node Boost::system)

# ── emotion_input_node ────────────────────────────────────────────────────────
# Toetsenbordinput → publiceert emotie string naar Raspberry Pi
add_executable(emotion_input_node src/emotion_input_node.cpp)
ament_target_dependencies(emotion_input_node rclcpp std_msgs)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
    quest_bridge_node
    arduino_servo_node
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
