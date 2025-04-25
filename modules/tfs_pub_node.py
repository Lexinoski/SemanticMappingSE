# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np



def main():

    rospy.init_node('tf2_control_node')
    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # 10 Hz

    try:
        while not rospy.is_shutdown():

            # # Translation and rotation from cesto to camera RGB.
            # x = 0
            # y = 0.5       Translation and rotation from cesto to camera RGB.
            # x = 0
            # y = 0.5
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = 0
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula
            
            # child_frame_id = "/camera"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)


            # # Translation and rotation from cesto to lidar 1.
            # x = -0.3
            # y = 0.4
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = np.pi/3
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula
            # child_frame_id = "/Lidar"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)

            # # Translation and rotation from cesto to lidar 2.
            # x = 0.3
            # y = 0.4
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = -np.pi/3
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula

            # child_frame_id = "/lidar2"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = 0
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula
            
            # child_frame_id = "/camera"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)


            # # Translation and rotation from cesto to lidar 1.
            # x = -0.3
            # y = 0.4
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = np.pi/3
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula
            # child_frame_id = "/Lidar"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)

            # Translation and rotation from cesto to lidar 2.
            # x = 0.3
            # y = 0.4
            # z = 0.5
            # translation = (x, y, z)
            # roll_angle = 0
            # pitch_angle = 0
            # yaw_angle = -np.pi/3
            # rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula

            # child_frame_id = "/lidar2"
            # transform = make_tf(translation, rotation, child_frame_id)
            # tf_broadcaster.sendTransform(transform)


            # Translation and rotation to the map.
            x = -111
            y = -115
            z = 0
            translation = (x, y, z)
            roll_angle = 0
            pitch_angle = 0
            yaw_angle = 0
            rotation = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)#(0, 0, 0, 1)  # Quaternion para rotação nula

            child_frame_id = "/map"
            transform = make_tf(translation, rotation, child_frame_id)
            tf_broadcaster.sendTransform(transform)


            rate.sleep()

    except rospy.ROSException as e:
        rospy.logwarn("Caught ROSException: {}. Retrying...".format(str(e)))



def make_tf(translation, rotation, child_frame_id):

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "/odom"
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]

    return transform


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converte ângulos de Euler para quaternion usando a convenção XYZ (Roll, Pitch, Yaw).
    
    :param roll: Ângulo de rotação em torno do eixo X (roll), em radianos.
    :param pitch: Ângulo de rotação em torno do eixo Y (pitch), em radianos.
    :param yaw: Ângulo de rotação em torno do eixo Z (yaw), em radianos.
    :return: Quaternion como uma lista [x, y, z, w].
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]
        
if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            main()
        except rospy.ROSException as e:
            rospy.logwarn("Caught ROSException: {}. Retrying...".format(str(e)))
            
    



