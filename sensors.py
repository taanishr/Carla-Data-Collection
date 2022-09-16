from constants import *
import carla
import queue

def setup_sensors(blueprint_library):

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

    return camera_blueprint, lidar_blueprint, depth_cam_blueprint

def generate_sensors(world, camera_blueprint, lidar_blueprint, depth_cam_blueprint):
    
    int_sensor1_transform = carla.Transform(carla.Location(x=114.093, y=29.769, z=5.853), carla.Rotation(yaw=180))
    int_sensor2_transform = carla.Transform(carla.Location(x=104.236, y=22.047, z=5.853), carla.Rotation(yaw=135))

    int_cam1 = world.spawn_actor(camera_blueprint, int_sensor1_transform)
    int_cam2 = world.spawn_actor(camera_blueprint, int_sensor2_transform)
    depth_cam = world.spawn_actor(depth_cam_blueprint, int_sensor1_transform)
    int_lidar1 = world.spawn_actor(lidar_blueprint, int_sensor1_transform)

    int_cam1_queue = queue.Queue()
    int_cam2_queue = queue.Queue()
    int_depth_cam_queue = queue.Queue()
    int_lidar1_queue = queue.Queue()

    int_cam1.listen(int_cam1_queue.put)
    int_cam2.listen(int_cam2_queue.put)
    depth_cam.listen(int_depth_cam_queue.put)
    int_lidar1.listen(int_lidar1_queue.put)

    return int_cam1, int_cam2, depth_cam, int_lidar1, int_cam1_queue, int_cam2_queue, int_depth_cam_queue, int_lidar1_queue