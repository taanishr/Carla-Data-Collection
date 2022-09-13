from turtle import distance
from uuid import NAMESPACE_OID
import numpy
import carla
from datetime import datetime
import os
import time
import queue
import math
from GenerateLabelFile import Label
from GenerateCalibFile import Calib

from GenerateBoundingBoxes import GenerateBoundingBoxes
from Vehicle import *
import numpy as np
from math import cos, sin
from utils import degrees_to_radians, transform_lidar

from constants import *
from dataexport import save_lidar_data

#from carla.Transform import *

# Create lidar path
if not os.path.exists(LIDAR_PATH):
    os.mkdir(LIDAR_PATH)

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

# Set up camera attached to ego vehicle
camera_blueprint = blueprint_library.find('sensor.camera.rgb')
camera_blueprint.set_attribute('image_size_x', str(WINDOW_WIDTH))
camera_blueprint.set_attribute('image_size_y', str(WINDOW_HEIGHT))

# Set up lidar sensor attached to ego vehicle
lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
lidar_blueprint.set_attribute('channels', '40')
lidar_blueprint.set_attribute('points_per_second', '720000')
lidar_blueprint.set_attribute('rotation_frequency', '10')
lidar_blueprint.set_attribute('range', str(MAX_RENDER_DEPTH_IN_METERS))
lidar_blueprint.set_attribute('lower_fov', str(-16))
lidar_blueprint.set_attribute('upper_fov', str(7))
#lidar_transform = carla.Transform(carla.Location(z=2))

# Set up depth camera
depth_cam_blueprint = blueprint_library.find("sensor.camera.depth")
depth_cam_blueprint.set_attribute('image_size_x', str(WINDOW_WIDTH))
depth_cam_blueprint.set_attribute('image_size_y', str(WINDOW_HEIGHT))
depth_cam_blueprint.set_attribute('fov', str(90.0))

# Spawn ego vehicle
vehicle_blueprint = blueprint_library.filter('model3')[0]

# Spawn vehicles
vehicle_spawn_point = carla.Transform(carla.Location(x=80, y=27.83, z=1))
vehicles = []
ego_vehicle = EgoVehicle(world, vehicle_blueprint, 'car', vehicle_spawn_point, tm)
ego_vehicle.enable_driving()
vehicles.append(ego_vehicle)

# Generate 3 sensors at intersection (2 RGB cams, 1 depth cam)
int_sensor1_transform = carla.Transform(carla.Location(x=114.093, y=29.769, z=5.853), carla.Rotation(yaw=180))
int_sensor2_transform = carla.Transform(carla.Location(x=104.236, y=22.047, z=5.853), carla.Rotation(yaw=135))

int_cam1 = world.spawn_actor(camera_blueprint, int_sensor1_transform)
int_cam2 = world.spawn_actor(camera_blueprint, int_sensor2_transform)
depth_cam = world.spawn_actor(depth_cam_blueprint, int_sensor1_transform)

int_cam1_queue = queue.Queue()
int_cam2_queue = queue.Queue()
int_depth_cam_queue = queue.Queue()

int_cam1.listen(int_cam1_queue.put)
int_cam2.listen(int_cam2_queue.put)
depth_cam.listen(int_depth_cam_queue.put)

# Generate 1 lidar sensor
int_lidar1 = world.spawn_actor(lidar_blueprint, int_sensor1_transform)
int_lidar1_queue = queue.Queue()
int_lidar1.listen(int_lidar1_queue.put)

# Generate camera matrix
camera_matrix = numpy.array(int_cam1.get_transform().get_inverse_matrix())

# Generate projection matrix from camera intrinsics
projection_matrix = GenerateBoundingBoxes.build_projection_matrix(WINDOW_WIDTH, WINDOW_HEIGHT)

# Get camera and lidar sensor transforms
camera_to_car_transform = int_cam1.get_transform()
#lidar_to_car_transform = int_lidar1.get_transform() * Transform(Rotation(yaw=90), Scale(z=-1))

# Creates Label data
def createLabelData(actor_name, ego_actor):
    # loop through all vehicles in world
    for npc in world.get_actors().filter('*vehicle*'):
        # check if vehicle isn't the same as ego actor
        if npc.id != ego_actor.id:
            # confirm npc vehicle is within 50 meters
            dist = npc.get_transform().location.distance(ego_actor.get_transform().location)
            if dist < 50:
                # determine if vehicle is in front of the camera
                forward_vec = ego_actor.get_transform().get_forward_vector()
                ray = npc.get_transform().location - ego_actor.get_transform().location
                if forward_vec.dot(ray) > 1:
                    # create state object
                    label = Label()
                    # open file
                    label_data_path = os.path.abspath(".\\labels.txt")
                    # f = open(label_data_path, 'a')

                    # Record vehicle type
                    for vehicle in vehicles:
                        if vehicle.get_id() == npc.id:
                            label.class_name = vehicle.type 
                        else:
                            label = 'DontCare'

                    # Record location and dimensions of vehicle
                    label.location = [npc.bounding_box.location.x, npc.bounding_box.location.y, npc.bounding_box.location.z]

                    # write 2d bounding boxes seen from ego_actor to file
                    Bounding_Boxes = GenerateBoundingBoxes(npc, projection_matrix, camera_matrix)

                    x_max, x_min, y_max, y_min = Bounding_Boxes.build2dBoundingBox()
                    if x_min > 0 and x_max < WINDOW_WIDTH and y_min > 0 and y_max < WINDOW_HEIGHT: 
                        label.bounding_box = [x_max, x_min, y_max, y_min]

                    # TODO: Calculate occlusion and truncation

                    label.dimensions = [float(npc.bounding_box.extent.x * 2), float(npc.bounding_box.extent.y * 2), float(npc.bounding_box.extent.z * 2)]

                    # Record camera angle
                    label.rotation_y = ego_actor.get_transform().rotation.yaw
                    
                    #     f.write(f'2D Bounding Box from view of ' + actor_name + ': ' + str([x_max, x_min, y_max, y_min]) + '\n')
                    
                    # f.close()

def createCalibData(WINDOW_HEIGHT, WINDOW_WIDTH):
    calib = Calib(WINDOW_HEIGHT, WINDOW_WIDTH)
    calib.save_calib_matrix("calib.txt")

# set up file system
os.chdir("..\\")

# start server tick
tick = 0

while True:
    world.tick()
    tick += 1

    # if (tick % 10 == 0):
        # get current date and time
        # current_date_time = datetime.now().strftime("%m-%d-%Y, %H-%M-%S")

        # change directory to sub folder
    path = os.path.abspath(".\\") + f"\\{tick}"
    os.mkdir(path)
    os.chdir(path)

    #measurements, sensor_data = client.read_data()

    #world_transform = Transform(
    #        measurements.player_measurements.transform
    #    )

    # save intersection camera data
    int_cam1_path = os.path.abspath(".\\INT_CAMERA1")
    int_cam2_path = os.path.abspath(".\\INT_CAMERA2")
    int_cam1_image = int_cam1_queue.get()
    int_cam2_image = int_cam2_queue.get()
    int_cam1_image.save_to_disk(int_cam1_path)
    int_cam2_image.save_to_disk(int_cam2_path)

    # save intersection lidar data
    lidar_path = os.path.abspath(".\\INT_LIDAR1")
    lidar_data = int_lidar1_queue.get()
    
    # Save lidar as bin file
    save_lidar_data(f"{LIDAR_PATH}/{tick}.bin", transform_lidar(lidar_data, int_lidar1, int_cam1))

    # get camera matrix
    camera_matrix = numpy.array(int_cam1.get_transform().get_inverse_matrix())
    
    createLabelData("int_cam1", int_cam1)

    # TODO: Save Label data to file

    createCalibData(WINDOW_HEIGHT, WINDOW_WIDTH)

    os.chdir("..\\")
