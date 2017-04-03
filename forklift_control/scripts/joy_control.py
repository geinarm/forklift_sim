#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

PUBLISH_RATE = 10 # 10hz
NODE_NAME = 'joy_control'
WHEEL_CONTROL_TOPIC = '/forklift/front_wheel_controller/cmd_vel'
LIFT_CONTROL_TOPIC = '/forklift/lift_controller/command'
JOYSTICK_TOPIC = '/joy'

LENGTH = 1.66
MAX_TURN = math.pi/4

class JoyControl:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)

        self.rate = rospy.Rate(PUBLISH_RATE)
        self.vertical = 0
        self.horizontal = 0
        self.lift = 0

        self.wheel_cmd_publisher = rospy.Publisher(WHEEL_CONTROL_TOPIC, Twist, queue_size=1)
        self.lift_cmd_publisher = rospy.Publisher(LIFT_CONTROL_TOPIC, Float64, queue_size=1)

        rospy.Subscriber(JOYSTICK_TOPIC, Joy, self.joy_callback)


    def joy_callback(self, data):

        self.vertical = data.axes[4]
        self.horizontal = data.axes[3]
        self.lift = data.axes[1]


    def run(self):
        while not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear = Vector3(self.vertical, 0, 0)

            turn = self.horizontal * MAX_TURN
            angular = (self.vertical/LENGTH) * math.tan(turn)
            cmd.angular = Vector3(0, 0, angular)

            self.wheel_cmd_publisher.publish(cmd)
            self.lift_cmd_publisher.publish(self.lift)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        joy_control = JoyControl()
        joy_control.run()
        
    except rospy.ROSInterruptException:
        pass