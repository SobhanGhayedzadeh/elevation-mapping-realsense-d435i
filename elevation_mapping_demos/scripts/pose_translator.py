#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf

def callback(newPose):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_listener

    # Create and fill pose message for publishing
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = "base_link"
    pose.pose.pose.position.x = 0
    pose.pose.pose.position.y = 0
    pose.pose.pose.position.z = 0
    pose.pose.pose.orientation.x = 0
    pose.pose.pose.orientation.y = 0
    pose.pose.pose.orientation.z = 0
    pose.pose.pose.orientation.w = 0

    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0]

    publisher.publish(pose)


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, tf_listener
    rospy.init_node('pose_translator')


    tf_listener = tf.TransformListener()
    publisher = rospy.Publisher(
        "/pose" , geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

    # Set callback and start spinning
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass























# PKG = 'pose_translator'
# import roslib; roslib.load_manifest(PKG)
# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped

# pub = rospy.Publisher('/pose', PoseWithCovarianceStamped, queue_size=200)


# def callback(data):
#     header = data.header
#     pose_w_c = data.pose
#     msg = PoseWithCovarianceStamped()
#     msg.header = header
#     msg.pose = pose_w_c
#     pub.publish(msg)
    
    
# def listener():
#     rospy.init_node('pose_translator')
#     rospy.Subscriber("/imu_odom", Odometry, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()