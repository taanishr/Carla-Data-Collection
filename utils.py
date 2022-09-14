import math
import numpy as np
import os
from constants import *

def degrees_to_radians(degrees):
    return degrees * math.pi / 180

def create_kitti_file_structure():

    """
    Creates KITTI file structure
    """

    if not os.path.exists(MAIN_FOLDER):
        os.mkdir(MAIN_FOLDER)
    
    if not os.path.exists(TESTING_FOLDER):
        os.mkdir(TESTING_FOLDER)
    
    if not os.path.exists(TRAINING_FOLDER):
        os.mkdir(TRAINING_FOLDER)
    
    if not os.path.exists(TESTING_CALIB):
        os.mkdir(TESTING_CALIB)
    
    if not os.path.exists(TRAINING_CALIB):
        os.mkdir(TRAINING_CALIB)

    if not os.path.exists(TESTING_IMAGE):
        os.mkdir(TESTING_IMAGE)
    
    if not os.path.exists(TRAINING_IMAGE):
        os.mkdir(TRAINING_IMAGE)
    
    if not os.path.exists(TESTING_LIDAR):
        os.mkdir(TESTING_LIDAR)
    
    if not os.path.exists(TRAINING_LIDAR):
        os.mkdir(TRAINING_LIDAR)
    
    if not os.path.exists(TRAINING_LABELS):
        os.mkdir(TRAINING_LABELS)

def transform_lidar(lidar_data, lidar_sensor, camera_sensor):
    """
    Transforms point cloud into camera space, and into a format so that 
    it can be saved to a bin
    """
    """ lidar_location, lidar_rotation = int_lidar1.get_transform()
    pitch, yaw, roll = lidar_rotation.pitch, lidar_rotation.yaw, lidar_rotation.roll

    pitch = degrees_to_radians(pitch)
    roll = degrees_to_radians(roll)
    yaw = degrees_to_radians(yaw)

    # Rotation matrix for pitch
    rotP = np.array([[cos(pitch),            0,              sin(pitch)],
                     [0,            1,     0],
                     [-sin(pitch),            0,     cos(pitch)]])
                        
    # Rotation matrix for roll
    rotR = np.array([[1,            0,              0],
                    [0,            cos(roll),     -sin(roll)],
                    [0,            sin(roll),     cos(roll)]])

    # combined rotation matrix, must be in order roll, pitch, yaw
    rotRP = np.matmul(rotR, rotP)

    # Transform to camera coordinatess
    point_cloud = np.dot(int_sensor1_transform, lidar)
    point_cloud[:, 2] -= LIDAR_HEIGHT_POS
    point_cloud = np.matmul(rotRP, point_cloud.T).T """
    # Get lidar data and convert it to a numpy array.
    p_cloud_size = len(lidar_data)
    p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
    p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))
    intensity = np.array(p_cloud[:, 3])

    # In lidar sensor space
    local_lidar_points = np.array(p_cloud[:, :3]).T
    local_lidar_points = np.r_[
                local_lidar_points, [np.ones(local_lidar_points.shape[1])]]
    
    lidar_2_world = lidar_sensor.get_transform().get_matrix()

    # Transform the points from lidar space to world space.
    world_points = np.dot(lidar_2_world, local_lidar_points)

    # This (4, 4) matrix transforms the points from world to camera coordinates.
    world_2_camera = np.array(camera_sensor.get_transform().get_inverse_matrix())

    # Transform the points from world space to camera space.
    sensor_points = np.dot(world_2_camera, world_points)

    return sensor_points