#!/usr/bin/env python3

###########################################################
# BUT :                                                   #
#   estimer la pose (position et orientation) du robot    #
#   transformer les données LiDAR de coordonnées polaires #
#   à un nuage de points                                  #
#   passer du repère du robot à un repère fixe            #
#                                                         #
# Utilise l'encodeur, le LiDAR                            #
# Correspond au TP03, TP04                                #
###########################################################


import numpy as np
import rospy
from tf.transformations import quaternion_from_euler

# Type of input and output messages
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud
from turtlebot3_msgs.msg import SensorState


# Déjà complété dans le TP03
def coordinates_to_message(x, y, O, t):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, O)
    msg.header.stamp = t
    msg.header.frame_id = 'odom'
    return msg


# Déjà complété dans le TP04
PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]


class CartographyNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('cartography')

        # Initialize the node constants
        # (cf. TP03 page 2)
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_SEPARATION = 0.16
        self.PERIOD = 1 / 20

        # Initialize the node parameters
        self.x_odom, self.y_odom, self.O_odom = 0, 0, 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.v = 0
        self.cloud = []

        # Publisher to the output topics.
        self.lidar_pub = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
        self.pose_pub = rospy.Publisher('/pose_enco', PoseStamped, queue_size=10)

        # Subscribers to the input topic. self.callback is called when a message is received
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.odom_sub = rospy.Subscriber('/sensor_state', SensorState, self.callback_odom)
    

    def callback_scan(self, msg):

        def cartesian_coords():
            coords = []
            for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
                r = msg.ranges[i]
                if r < 0.3:
                    continue

                # Polar to Cartesian transformation
                # (formule TP04 page 1)
                x = r * np.cos(theta)
                y = r * np.sin(theta)
                coords.append((x, y, 1))
            
            return coords

        # Change base
        coords_odom = np.array(
            [
                [np.cos(self.O_odom), - np.sin(self.O_odom), self.x_odom],
                [np.sin(self.O_odom), np.cos(self.O_odom), self.y_odom],
                [0, 0, 1]
            ]
        )
        coords = coords_odom @ np.array(cartesian_coords()).T

        # Merge PointCloud
        self.cloud = self.cloud + [[x,y,0,0] for x, y, _ in coords.T]
        self.lidar_pub.publish(create_cloud(msg.header, PC2FIELDS, self.cloud))


    def callback_odom(self, sensor_state):
        # Compute the differential in encoder count
        d_left_encoder = sensor_state.left_encoder - self.prev_left_encoder
        d_right_encoder = sensor_state.right_encoder - self.prev_right_encoder

        # Treating the first info
        if self.prev_left_encoder == 0:
            self.prev_left_encoder = sensor_state.left_encoder
            self.prev_right_encoder = sensor_state.right_encoder
            return

        self.prev_right_encoder = sensor_state.right_encoder
        self.prev_left_encoder = sensor_state.left_encoder

        # Compute the linear and angular velocity (self.v and w)
        # (formule du cours 5 diapo 12)
        d_left_angle = (d_left_encoder * 20) * (2 * np.pi / self.ENCODER_RESOLUTION)
        d_right_angle = (d_right_encoder * 20) * (2 * np.pi / self.ENCODER_RESOLUTION)
        
        # (formule du cours 5 diapo 15)
        self.v = self.WHEEL_RADIUS * (d_left_angle + d_right_angle) / 2
        w = self.WHEEL_RADIUS * (d_right_angle - d_left_angle) / self.WHEEL_SEPARATION
        
        # Update x_odom, y_odom and O_odom accordingly
        # (formule du cours 5 diapo 8)
        self.x_odom += np.cos(self.O_odom) * (self.v / 20)
        self.y_odom += np.sin(self.O_odom) * (self.v / 20)
        self.O_odom += (w / 20)

        # Publish pose
        msg = coordinates_to_message(self.x_odom, self.y_odom, self.O_odom, sensor_state.header.stamp)
        self.pose_pub.publish(msg)


if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CartographyNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
