from math import pi
import os
from constants import WINDOW_HEIGHT, WINDOW_WIDTH
from GenerateBoundingBoxes import GenerateBoundingBoxes

class Label:
    def __init__(self):
        self.class_name = ''
        self.truncated = 0
        self.occluded = 0
        self.alpha = -10
        self.bbox = None
        self.dimensions = None
        self.location = None
        self.rotation_y = None
        self.extent = None
        
    def set_type(self, obj_type: str):
        self.type = obj_type
    
    def set_truncated(self, truncated: float):
        self.truncated = truncated
    
    def set_occlusion(self, occlusion: int):
        self._occluded = occlusion

    def set_alpha(self, alpha: float):
        assert -pi <= alpha <= pi, "Alpha must be in range [-pi..pi]"
        self.alpha = alpha

    def set_bbox(self, bbox):
        assert len(bbox) == 4, """ 
        Bbox must be 2D bounding box of object in the image (0-based index):
        contains left, top, right, bottom pixel coordinates (two points)
        """
        self.bbox = bbox

    def set_3d_object_dimensions(self, bbox_extent):
        # Bbox extent consists of x,y and z.
        # The bbox extent is by Carla set as
        # x: length of vehicle (driving direction)
        # y: to the right of the vehicle
        # z: up (direction of car roof)
        # However, Kitti expects height, width and length (z, y, x):
        height, width, length = bbox_extent.z, bbox_extent.x, bbox_extent.y
        # Since Carla gives us bbox extent, which is a half-box, multiply all by two
        self.extent = (height, width, length)
        self.dimensions = "{} {} {}".format(2*height, 2*width, 2*length)

    def set_3d_object_location(self, obj_location):
        """ 
            Converts the 3D object location from CARLA coordinates and saves them as KITTI coordinates in the object
            In Unreal, the coordinate system of the engine is defined as, which is the same as the lidar points
            z
            ▲   ▲ x
            |  /
            | /
            |/____> y
            This is a left-handed coordinate system, with x being forward, y to the right and z up 
            See also https://github.com/carla-simulator/carla/issues/498
            However, the camera coordinate system for KITTI is defined as
                ▲ z
               /
              /
             /____> x
            |
            |
            |
            ▼
            y 
            This is a right-handed coordinate system with z being forward, x to the right and y down
            Therefore, we have to make the following changes from Carla to Kitti
            Carla: X   Y   Z
            KITTI:-X  -Y   Z
        """
        
        x, y, z = [float(x) for x in obj_location]
        
        assert None not in [
            self.extent, self.type], "Extent and type must be set before location!"
        
        if self.class_name == "Pedestrian":
            # Because the midpoint/location of the pedestrian is in the middle of the agent, while for car it is at the bottom
            # we need to subtract the bbox extent in the height direction when adding location of pedestrian.
            y -= self.extent[0]
        
        self.location = " ".join(map(str, [y, -z, x]))
        

    def set_rotation_y(self, rotation_y: float):
        assert - \
            pi <= rotation_y <= pi, "Rotation y must be in range [-pi..pi] - found {}".format(
                rotation_y)
        self.rotation_y = rotation_y

    def __str__(self):
        """ Returns the kitti formatted string of the datapoint if it is valid (all critical variables filled out), else it returns an error."""
        if self.bbox is None:
            bbox_format = " "
        else:
            bbox_format = " ".join([str(x) for x in self.bbox])

        return "{} {} {} {} {} {} {} {}".format(self.type, self.truncated, self.occluded, self.alpha, bbox_format, self.dimensions, self.location, self.rotation_y)

# Creates Label data
def createLabelData(file_name, world, vehicles, projection_matrix, camera_matrix, ego_actor):
    
    f = open(file_name, 'a+')
    
    # Loop through all vehicles in world
    for npc in world.get_actors().filter('*vehicle*'):

        # Check if vehicle isn't the same as ego actor
        if npc.id != ego_actor.id:

            # Confirm npc vehicle is within 50 meters
            dist = npc.get_transform().location.distance(ego_actor.get_transform().location)

            # TODO: Calculate occlusion and truncation and replace dist heuristic

            if dist < 50:
                
                # Determine if vehicle is in front of the camera
                forward_vec = ego_actor.get_transform().get_forward_vector()
                ray = npc.get_transform().location - ego_actor.get_transform().location
                
                if forward_vec.dot(ray) > 1:
                    
                    label = Label()

                    # Record vehicle type. Currently only vehicles are spawned.
                    for vehicle in vehicles:
                        if vehicle.get_id() == npc.id:
                            label.class_name = "Car"

                    # Record location and dimensions of vehicle
                    label.set_3d_object_location((npc.bounding_box.location.x, npc.bounding_box.location.y, npc.bounding_box.location.z))

                    # write 2d bounding boxes seen from ego_actor to file
                    Bounding_Boxes = GenerateBoundingBoxes(npc, projection_matrix, camera_matrix)
                    x_max, x_min, y_max, y_min = Bounding_Boxes.build2dBoundingBox()
                    if x_min > 0 and x_max < WINDOW_WIDTH and y_min > 0 and y_max < WINDOW_HEIGHT: 
                        label.set_bbox((x_max, x_min, y_max, y_min))

                    # Set vehicle 3D object dimensions
                    bbox_extent = (float(npc.bounding_box.extent.z * 2), float(npc.bounding_box.extent.x * 2), float(npc.bounding_box.extent.y * 2))
                    label.set_3d_object_dimensions(bbox_extent)

                    # Record camera angle
                    label.rotation_y = ego_actor.get_transform().rotation.yaw
                    
                    f.write(label)  
    
    f.close()