#! /usr/bin/python
from dataclasses import dataclass
from math import atan2, pi

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


DISTANCE_TOLERANCE = float(rospy.get_param('distance'))
POLICE_SPEED = float(rospy.get_param('speed'))


@dataclass
class Position:
    x: float
    y: float


class TurtleBot:
    def __init__(self):
        rospy.init_node('my_turtle_bot_node')

        rospy.wait_for_service('/spawn')
        spawn_function = rospy.ServiceProxy('/spawn', Spawn)
        spawn_function(4.0, 4.0, 0.0, 'leo')

        self.publisher = rospy.Publisher('/leo/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(1)

        self.enemy_position = Position(0.0, 0.0)
        self.leo_position = Position(0.0, 0.0)

        self.pose = Pose()
        rospy.Subscriber('/turtle1/pose', Pose, self.enemy)
        rospy.Subscriber('/leo/pose', Pose, self.leo)


    def enemy(self, msg):
        self.enemy_position.x = msg.x
        self.enemy_position.y = msg.y

    def leo(self, msg):
        self.pose = msg
        self.leo_position.x = msg.x
        self.leo_position.y = msg.y

    def calculate_distance(self):
        return (
            (self.enemy_position.x - self.leo_position.x) ** 2 +
            (self.enemy_position.y - self.leo_position.y) ** 2
        ) ** 0.5

    def steering_angle(self):
        return atan2(
            self.enemy_position.y - self.leo_position.y,
            self.enemy_position.x - self.leo_position.x
        )
    
    def linear_vel(self, max_speed, angular_speed):
        if angular_speed > pi / 4:
            return 0
        return max_speed

    def angular_vel(self):
        angles = []
        angle =  self.steering_angle() - self.pose.theta
        angles.append(angle)
        angles.append(2 * pi - angle)
        angles.append(2 * pi + angle)

        abs_angles = list(map(abs, angles))
        return angles[abs_angles.index(min(abs_angles))]

    def follow(self):
        message = Twist()

        while not rospy.is_shutdown():
            if self.calculate_distance() > DISTANCE_TOLERANCE:
                message.angular.z = self.angular_vel()
                message.linear.x = self.linear_vel(POLICE_SPEED, message.angular.z)

                self.publisher.publish(message)

                self.rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.follow()
    except rospy.ROSInterruptException:
        pass