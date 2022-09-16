from turtle import distance
from uuid import NAMESPACE_OID
import numpy
import carla
from datetime import datetime
import os
import time
import queue
import math
from GenerateLabelFile import *
from GenerateCalibFile import *

from GenerateBoundingBoxes import GenerateBoundingBoxes
from Vehicle import *
import numpy as np
from math import cos, sin
from utils import *

from constants import *
from dataexport import save_lidar_data

from sensors import *

#from carla.Transform import *

# Set up file structure
create_kitti_file_structure()

# Variable to decide whether to save to testing or training folder. True for training, False otherwise
save_to_train = True

# Set up world
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set world to synchronous
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

# Get spawn points and blueprints
spawn_points = world.get_map().get_spawn_points()

# Set up blueprints
blueprint_library = world.get_blueprint_library()

# Enable traffic manager
tm = client.get_trafficmanager()
tm.set_synchronous_mode(True)

camera_blueprint, lidar_blueprint, depth_cam_blueprint = setup_sensors()

int_cam1, int_cam2, depth_cam,\
int_lidar1, int_cam1_queue, int_cam2_queue, \
int_depth_cam_queue, int_lidar1_queue = generate_sensors()

# Spawn ego vehicle
vehicle_blueprint = blueprint_library.filter('model3')[0]

# Spawn vehicles
vehicle_spawn_point = carla.Transform(carla.Location(x=100, y=40.83, z=1))
vehicles = []
ego_vehicle = EgoVehicle(world, vehicle_blueprint, 'car', vehicle_spawn_point, tm)
ego_vehicle.enable_driving()
vehicles.append(ego_vehicle)

# Generate camera matrix
camera_matrix = numpy.array(int_cam1.get_transform().get_inverse_matrix())

# Generate projection matrix from camera intrinsics
projection_matrix = GenerateBoundingBoxes.build_projection_matrix(WINDOW_WIDTH, WINDOW_HEIGHT)

# Get camera and lidar sensor transforms
camera_to_car_transform = int_cam1.get_transform()
#lidar_to_car_transform = int_lidar1.get_transform() * Transform(Rotation(yaw=90), Scale(z=-1))

# Start server tick
tick = 0

while True:
    
    world.tick()
    tick += 1

    #measurements, sensor_data = client.read_data()

    #world_transform = Transform(
    #        measurements.player_measurements.transform
    #    )

    # Deque data
    int_cam1_image = int_cam1_queue.get()
    int_cam2_image = int_cam2_queue.get()
    lidar_data = int_lidar1_queue.get()

    # Get camera matrix
    camera_matrix = numpy.array(int_cam1.get_transform().get_inverse_matrix())

    # Save to training if save_to_train is True, testing otherwise
    curr_folder = TRAINING_FOLDER if save_to_train else TESTING_FOLDER
    
    # Save data
    int_cam1_image.save_to_disk(f"{curr_folder}/tick")
    #int_cam2_image.save_to_disk(f"{curr_folder}/tick")

    save_lidar_data(f"{curr_folder}/velodyne/{tick}.bin", transform_lidar(lidar_data, int_lidar1, int_cam1))
    
    createCalibData(f"{curr_folder}/calib/{tick}.txt")

    # TODO: Save Label data to file
    createLabelData("int_cam1", int_cam1)

    # Flip save_to_train
    save_to_train = not save_to_train