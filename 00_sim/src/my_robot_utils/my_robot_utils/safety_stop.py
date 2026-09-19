#! /usr/bin/env python3

import sys
from enum import Enum
from math import isinf, isnan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0
    SLOW = 1
    STOP = 2


class SafetyStop(Node):

    def __init__(self):
        super().__init__("safety_stop_node")

        # 20 cm; robot should stop completely if any object is detected within this radius
        self.declare_parameter("danger_radius", 0.5)
        # when an object is detected within a 60cm radius the robot will slow
        # down; helps in reducing its inertia in case it needs to come to a halt
        self.declare_parameter("warning_radius", 1.5)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safety_stop_topic", "/safety_stop")

        self.danger_radius = self.get_parameter("danger_radius").get_parameter_value().double_value
        self.warning_radius = self.get_parameter(
            "warning_radius").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter(
            "safety_stop_topic").get_parameter_value().string_value

        self.state = State.FREE
        self.prev_state = State.FREE
        self.is_first_scan = True

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laserCb, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "/zones", 10)

        self.decrease_speed_cli = ActionClient(self, JoyTurbo, "/joy_turbo_decrease")
        self.increase_speed_cli = ActionClient(self, JoyTurbo, "/joy_turbo_increase")

        while not self.decrease_speed_cli.server_is_ready():
            if rclpy.ok():
                self.get_logger().info(
                    f"waiting for {
                        self.decrease_speed_cli._action_name} action server...")
            else:
                sys.exit(1)

        while not self.increase_speed_cli.server_is_ready():
            if rclpy.ok():
                self.get_logger().info(
                    f"waiting for {
                        self.increase_speed_cli._action_name} action server...")
            else:
                sys.exit(1)

        self.zones = MarkerArray()
        warning_zone = Marker()
        warning_zone.id = 1
        warning_zone.type = Marker.CYLINDER
        warning_zone.action = Marker.ADD
        warning_zone.scale.x = self.warning_radius * 2
        warning_zone.scale.y = self.warning_radius * 2
        warning_zone.scale.z = 0.002
        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.9
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.2

        danger_zone = Marker()
        danger_zone.id = 0
        danger_zone.type = Marker.CYLINDER
        danger_zone.action = Marker.ADD
        danger_zone.pose.position.z = 0.01
        danger_zone.scale.x = self.danger_radius * 2
        danger_zone.scale.y = self.danger_radius * 2
        danger_zone.scale.z = 0.002
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.2

        self.zones.markers = [danger_zone, warning_zone]

    def laserCb(self, msg: LaserScan):

        self.state = State.FREE

        is_safety_stop = Bool()

        for distance in msg.ranges:
            if not (isinf(distance) or isnan(distance)):
                if msg.range_min <= distance <= msg.range_max:
                    if distance <= self.warning_radius:
                        self.state = State.SLOW
                        if distance <= self.danger_radius:
                            self.state = State.STOP
                            break

        if self.state != self.prev_state:
            if self.state == State.SLOW:
                is_safety_stop.data = False
                self.decrease_speed_cli.send_goal_async(JoyTurbo.Goal())
                self.zones.markers[0].color.a = 0.2
                self.zones.markers[1].color.a = 0.7
            elif self.state == State.STOP:
                is_safety_stop.data = True
                self.zones.markers[0].color.a = 0.7
                self.zones.markers[1].color.a = 0.7
            elif self.state == State.FREE:
                is_safety_stop.data = False
                self.increase_speed_cli.send_goal_async(JoyTurbo.Goal())
                self.zones.markers[0].color.a = 0.2
                self.zones.markers[1].color.a = 0.2

            self.prev_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)

        if self.is_first_scan:
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id
                zone.frame_locked = True

            self.is_first_scan = False

        self.zones_pub.publish(self.zones)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
