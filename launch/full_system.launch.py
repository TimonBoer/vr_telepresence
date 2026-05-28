from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── quest_bridge_node ──────────────────────────────────────────────────
        # Ontvangt TCP JSON van de Meta Quest (poort 5005)
        # Publiceert op: /head/orientation (geometry_msgs/QuaternionStamped)
        Node(
            package='vr_telepresence',
            executable='quest_bridge_node',
            name='quest_bridge',
            output='screen',
        ),

        # ── arduino_servo_node ─────────────────────────────────────────────────
        # Luistert naar: /head/orientation
        # Converteert quaternion → Euler → servo bytes
        # Stuurt naar Arduino via /dev/ttyACM0
        Node(
            package='vr_telepresence',
            executable='arduino_servo_node',
            name='arduino_servo',
            output='screen',
        ),

        # ── emotion_input_node ─────────────────────────────────────────────────
        # Toetsenbordinput → /robot/expression (std_msgs/String)
        # Opent een eigen terminal omdat deze node de keyboard blokkeert
        Node(
            package='vr_telepresence',
            executable='emotion_input_node',
            name='emotion_input',
            output='screen',
            prefix='xterm -e',
        ),

    ])
