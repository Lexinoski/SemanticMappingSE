# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from time import sleep
import pandas as pd
# import cv2

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


from std_msgs.msg import Float32MultiArray

import tf2_ros
from geometry_msgs.msg import TransformStamped
import sys

# import quaternion


# Define a function to convert from pixels from algles theta phi;
def det_pixels_to_angles(det_pixels):

    # Set resolution of the image
    resolution = (640, 480)

    # Set view angles capacities of the cam
    view_angles = (90*(np.pi/180), 90*(np.pi/180))

    det_angles = list()
    for i in range(0, len(det_pixels)):
        # Update here to use the transforme.rotation
        angles = [np.interp(det_pixels[i][0], [0, resolution[0]], [90*(np.pi/180)-view_angles[0]/2, 90*(np.pi/180)+view_angles[0]/2]),
                  np.interp(det_pixels[i][1], [0, resolution[1]], [90*(np.pi/180)-view_angles[1]/2, 90*(np.pi/180)+view_angles[1]/2]),
                  np.interp(det_pixels[i][2], [0, resolution[0]], [90*(np.pi/180)-view_angles[0]/2, 90*(np.pi/180)+view_angles[0]/2]),
                  np.interp(det_pixels[i][3], [0, resolution[1]], [90*(np.pi/180)-view_angles[1]/2, 90*(np.pi/180)+view_angles[1]/2]),
                  det_pixels[i][4], det_pixels[i][5]]      
        det_angles.append(angles)
    return det_angles


# Convert quartenion to rotation matriz
def quaternion_to_rotation_matrix(q):
    """
    Converts a quaternion to a 3x3 rotation matrix.

    Args:
        q (list or np.array): A quaternion represented as a list or NumPy array [w, x, y, z].

    Returns:
        np.array: The 3x3 rotation matrix corresponding to the quaternion.
    """
    w, x, y, z = q
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return rotation_matrix


# Fuction for transfor points from lidar to cam rgb and odom coordinates
def transform_points(points, transform):
    """
    Transforms lidar points to camera RGB coordinates using a given transformation.

    Args:
        lidar_points (np.array): Lidar points represented as a numpy array of shape (N, 3).
        transform (dict): A dictionary containing the transformation information.

    Returns:
        np.array: Transformed points in camera RGB coordinates as a numpy array of shape (N, 3).
    """
    # Extract translation
    translation = np.array([transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z])

    # Extract quaternion of the rotation components from the dictionary
    rotation_w = transform.transform.rotation.w
    rotation_x = transform.transform.rotation.x
    rotation_y = transform.transform.rotation.y
    rotation_z = transform.transform.rotation.z

    # Constructing the quaternion from the extracted values
    q = [rotation_w, rotation_x, rotation_y, rotation_z]

    # Converting the quaternion to a rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(q)

    # print('LEN POINTS: {}'.format(len(points)))

    if len(points) > 3:
        # Build the homogeneous transformation matrix, starting by identity matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        # Apply the transformation points from lidar to rgb cam
        transformed_points = np.dot(transformation_matrix, np.hstack([points, np.ones((points.shape[0], 1))]).T).T[:, :3]

        return transformed_points
    else:
        # Apply rotation to the point
        rotated_point = np.dot(rotation_matrix, points)
        
        # Apply translation
        transformed_point = rotated_point + translation
        
        return transformed_point
        

# Define a function for convert quartenions to euler
# def quaternion_to_euler(quaternion):
#     # Extract components of the quaternion
#     w, x, y, z = quaternion

#     # Compute the Euler angles
#     roll = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x**2 + y**2))
#     pitch = np.arcsin(2 * (w*y - z*x))
#     yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y**2 + z**2))

#     return roll, pitch, yaw


# Function to convert from x, y and z to r, theta and phi coordinates.
def cartesian_to_spherical(x,y,z):
    r = []
    theta = []
    phi = []
    for i in range(len(x)):
        r.append(np.sqrt(np.power(x[i],2) + np.power(y[i],2) + np.power(z[i],2)))
        theta.append(np.arccos(y[i]/r[i])) # y axies is the heigth
        phi.append(np.arccos(x[i]/(r[i]*np.sin(theta[i]))))

    return r, theta, phi

def show_deep(depth):
    cv2.imshow("Depth Window", depth)
    cv2.waitKey(10) & 0xFF  # Aguarda por 10 milissegundos e verifica se alguma tecla é pressionada
    # No Python 2, cv2.waitKey() retorna um número inteiro, mas apenas os 8 bits menos significativos são usados para representar o código da tecla.
    # Portanto, aplicamos a máscara 0xFF para obter apenas esses bits.


# Depth analisys function
def depth_analisys():
    
    #### ----- FREZED ----- ######################
    ############# Set downsampling rate
    ds = 8
    ############# Define the center rate for bouding box
    center_rate = 0.7

    ############# Set height limits to be considered from camera at 0.6m
    height_min = -0.40
    height_max = + 10.0

    ############# Define number of points to compute positions
    percent_points = 10
    max_points = 10000
    min_points = 1

    #### ----- FREZED ----- ######################

    ############# Set 1 to show depths
    show_depths = 0

    rospy.init_node('depth_analisys', anonymous=True)
    # Init tfs
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # depth_pub = rospy.Publisher('depths_BB', Float32MultiArray, queue_size=10)
    positions_pub = rospy.Publisher('positions', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        # Wait for message from YOLOv7 detections
        dets_yolov7 = rospy.wait_for_message("/detections", Float32MultiArray)

        # Waiting for the point cloud message
        data = rospy.wait_for_message("/cloud_out", PointCloud2)

        # Adquire the transform from lidar to camera
        tf_lidar_to_cam = tf_buffer.lookup_transform('camera','camera', rospy.Time(0), rospy.Duration(1.0))
        # Adquire the transform from camera to odom
        tf_cam_to_odom = tf_buffer.lookup_transform('odom', 'camera', rospy.Time(0), rospy.Duration(1.0))

         # Deserialize data
        bboxes = [[float(dets_yolov7.data[6*i+j]) for j in range(0, 6)] for i in range(0,int(len(dets_yolov7.data)/6))]
        # Convert the pixels values to angles according cam specs. Here is considered resolution in xy axies and view angles of the camera.
        bboxes_angles = det_pixels_to_angles(bboxes)
        # Turn to array
        bboxes_angles = np.array(bboxes_angles)

        # Getting only x, y, and z, disregarding rgb with field_names=("x", "y", "z")
        cloud_xyzList = pc2.read_points_list(data, field_names=("x", "y", "z"))
        # Flipping the height because the point cloud starts from the bottom left and YOLOv7 bounding box considers p(0,0) at the top left.
        # And applying the Downsample here, as soon as possible
        cloud_xyzArray_ds = np.array([cloud_xyzList[640*i+j] for i in range(480-1, -1,-ds) for j in range(0,640,ds)])
        # Applying lidar to camera RGB transformation to the entire list of points
        cloud_xyzArray_ds_cam = transform_points(cloud_xyzArray_ds, tf_lidar_to_cam)


        # Get x, y and z coordenates of pc2
        x_ds = cloud_xyzArray_ds_cam[:, 0]
        y_ds = cloud_xyzArray_ds_cam[:, 1]
        z_ds = cloud_xyzArray_ds_cam[:, 2]

        # Convert from x, y and z to r, theta and phi coordinates.
        r_ds, theta_ds, phi_ds = cartesian_to_spherical(x_ds,y_ds,z_ds)

        # PolarPoints_ds = zip(r_ds, theta_ds, phi_ds)

        # Create a 2D Depth image, register max depth to 255 values
        # max_deep = max(r)
        # print("Max deep: {}" .format(max_deep))
        # DepthImage_ds = np.array([[int(round(255* r[int(row*640/ds) + col]/max_deep)) for col in range(0, int(640/ds))] for row in range(0, int(480/ds))])                                    
        # DepthImage_ds = DepthImage_ds.astype('uint8')
        # print('Deep image finish!')

        # # Criação da figura e do subplot 3D
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        # # Plotagem dos pontos
        # ax.scatter(x_ds, y_ds, z_ds)

        # # Rótulos dos eixos
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')

        # # Exibição do gráfico
        # plt.show()
       
        # print('\nStart a new sample')
        distances = list()
        positions = list()

        for i in range(0,len(bboxes_angles)):
            
            # Auxiliary print
            # print("Detection[{}]: {}" .format(i, bboxes_angles[i]))

            # Get images delimited by BBs. Apply heigh filtering here
            # Here is considered resolution in xy axies in the conversion to spherical coordenates
            ind_filtred = np.array([j for j in range(0, len(r_ds))
                                    if phi_ds[j] > bboxes_angles[i][0]+(((bboxes_angles[i][2]-bboxes_angles[i][0])*(1-center_rate))/2)
                                    and phi_ds[j] < bboxes_angles[i][2]-(((bboxes_angles[i][2]-bboxes_angles[i][0])*(1-center_rate))/2)
                                    and theta_ds[j] > bboxes_angles[i][1]+(((bboxes_angles[i][3]-bboxes_angles[i][1])*(1-center_rate))/2)
                                    and theta_ds[j] < bboxes_angles[i][3]-(((bboxes_angles[i][3]-bboxes_angles[i][1])*(1-center_rate))/2)
                                    and y_ds[j] > height_min and y_ds[j] < height_max])    # This line is the height filtering.
            
            cloud_xyzArray_ds_cam_filtred = np.array([cloud_xyzArray_ds_cam[ind_filtred[j]] for j in range(0, len(ind_filtred))])
            r_filtred = np.array([r_ds[ind_filtred[j]] for j in range(0, len(ind_filtred))])

            # print(np.array([theta_ds[ind_filtred[j]] for j in range(0, len(ind_filtred))]))

            # Get indices of the depth sorted
            indices_ordem = np.argsort(r_filtred)

            # Ordene data serialized according indices sorded by depth nearest
            cloud_xyzArray_ds_cam_filtred = cloud_xyzArray_ds_cam_filtred[indices_ordem]
            r_filtred = r_filtred[indices_ordem]


            # Get n points
            cloud_xyzArray_ds_cam_filtred = cloud_xyzArray_ds_cam_filtred[0: max(min(int(len(r_filtred)*percent_points/100), min_points), max_points)]
            r_filtred = r_filtred[0: max(min(int(len(r_filtred)*percent_points/100), min_points), max_points)]


            # print('SHAPE1: {}'.format(np.array(cloud_xyzArray_ds_cam_filtred).shape))
            # print('SHAPE2: {}'.format(r_filtred.shape))



            # Show image depth if show_depths sets to 1
            if show_depths:
                print('Size BBox {}: {}' .format(i+1, np.shape(ImageCroped)))
                show_deep(cv2.resize(ImageCroped, (640, 480), interpolation = cv2.INTER_AREA))
                sleep(1)
            

            # # Create list of distante to return to YOLOv7 node.
            distances.append((np.mean(r_filtred)))


            # Check if there is overlap for each objects
            if len(bboxes_angles) > 1:
                for j in range(0,len(bboxes_angles)):
                    if i != j:
                        if (bboxes_angles[i][0] < bboxes_angles[j][2] and bboxes_angles[i][2] > bboxes_angles[j][0] and
                         bboxes_angles[i][1] < bboxes_angles[j][3] and bboxes_angles[i][3] > bboxes_angles[j][1]):
                            overlap = True

                        else:
                            overlap = False

            # If alone there is not overlap all right
            else:
                overlap = False

            # Create position list to return to new algorithm node, if there is not overlap.
            if len(cloud_xyzArray_ds_cam_filtred) > 0 and not overlap:
                # cloud_xyzArray_ds_cam_filtred = np.array(cloud_xyzArray_ds_cam_filtred[0:n_points])

                x_mean = np.mean(cloud_xyzArray_ds_cam_filtred[:,0])
                y_mean = np.mean(cloud_xyzArray_ds_cam_filtred[:,1])
                z_mean = np.mean(cloud_xyzArray_ds_cam_filtred[:,2])
                
                position_mean_to_odom = transform_points(np.array([x_mean, y_mean, z_mean]), tf_cam_to_odom)
                positions.append(np.array([position_mean_to_odom[0], position_mean_to_odom[1], position_mean_to_odom[2], bboxes_angles[i][4], bboxes_angles[i][5]]))

        # Print the results
        # print('Distances: {}' .format(distances))
        # print('Positions: {}'.format(positions))


        # Publish ROS topics, depth first (10 % near points as published in the Article)
        # depths_msg = Float32MultiArray()
        #     # Get n points
        #     cloud_xyzArray_ds_cam_filtred = cloud_xyzArray_ds_cam_filtred[0:int(len(r_filtred)/10)]
        #     r_filtred = r_filtred[0:int(len(r_filtred)/10)]
        # depths_msg.data = distances
        # # rospy.loginfo(my_msg)
        # depth_pub.publish(depths_msg)


        # Serialize positions data
        positions_serialized = [float(positions[i][j]) for i in range(len(positions)) for j in range(len(positions[0]))]

        # Publish octo_cropped_boxes in topic ROS
        positions_msg = Float32MultiArray()
        positions_msg.data = positions_serialized
        positions_pub.publish(positions_msg)



if __name__ == '__main__':
    try:
        depth_analisys()
        print('DepthEstimation node was initialized')
    except rospy.ROSInterruptException:
        pass
