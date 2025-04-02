#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class OmniWheelController:
    def __init__(self):
        rospy.init_node('omni_wheel_controller', anonymous=True)

        self.pub_b1 = rospy.Publisher('/my_robot/joint_b1_velocity_controller/command', Float64, queue_size=10)
        self.pub_b2 = rospy.Publisher('/my_robot/joint_b2_velocity_controller/command', Float64, queue_size=10)
        self.pub_b3 = rospy.Publisher('/my_robot/joint_b3_velocity_controller/command', Float64, queue_size=10)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.wheel_radius = 0.05
        self.wheel_distance = 0.1

        # Chờ tất cả các publisher kết nối
        rate = rospy.Rate(1)
        while (self.pub_b1.get_num_connections() == 1 or 
               self.pub_b2.get_num_connections() == 1 or 
               self.pub_b3.get_num_connections() == 1) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for Gazebo to connect to joint commands...")
            rate.sleep()
        rospy.loginfo("All joint command topics are ready!")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        v_b1 = (linear_x - angular_z * self.wheel_distance) / self.wheel_radius
        v_b2 = (linear_x * (-0.5) + linear_y * (0.866) - angular_z * self.wheel_distance) / self.wheel_radius
        v_b3 = (linear_x * (-0.5) + linear_y * (-0.866) - angular_z * self.wheel_distance) / self.wheel_radius

        self.pub_b1.publish(v_b1)
        self.pub_b2.publish(v_b2)
        self.pub_b3.publish(v_b3)

        rospy.loginfo("Wheel velocities: b1=%.2f, b2=%.2f, b3=%.2f", v_b1, v_b2, v_b3)

if __name__ == '__main__':
    try:
        controller = OmniWheelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass