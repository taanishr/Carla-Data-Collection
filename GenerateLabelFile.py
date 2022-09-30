from math import pi, acos, radians
from constants import WINDOW_HEIGHT, WINDOW_WIDTH
from GenerateBoundingBoxes import GenerateBoundingBoxes

"""
#Values    Name      Description
----------------------------------------------------------------------------
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
----------------------------------------------------------------------------
"""

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
        
    def set_class_name(self, obj_type: str):
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
        height, width, length = bbox_extent[0], bbox_extent[1], bbox_extent[2]
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
            self.extent, self.class_name], "Extent and type must be set before location!"
        
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

        return "{} {} {} {} {} {} {} {}".format(self.class_name, self.truncated, self.occluded, self.alpha, bbox_format, self.dimensions, self.location, self.rotation_y)

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
                            label.set_class_name("Car")

                    # write 2d bounding boxes seen from ego_actor to file
                    Bounding_Boxes = GenerateBoundingBoxes(npc, projection_matrix, camera_matrix)
                    x_max, x_min, y_max, y_min = Bounding_Boxes.build2dBoundingBox()
                    if x_min > 0 and x_max < WINDOW_WIDTH and y_min > 0 and y_max < WINDOW_HEIGHT: 
                        label.set_bbox((x_max, x_min, y_max, y_min))

                    # Set vehicle 3D object dimensions and extent
                    bbox_extent = (float(npc.bounding_box.extent.z * 2), float(npc.bounding_box.extent.x * 2), float(npc.bounding_box.extent.y * 2))
                    label.set_3d_object_dimensions(bbox_extent)

                    # Record location and dimensions of vehicle
                    label.set_3d_object_location((npc.bounding_box.location.x, npc.bounding_box.location.y, npc.bounding_box.location.z))

                    # Record camera angle
                    label.set_rotation_y(radians(ego_actor.get_transform().rotation.yaw))
                    
                    # Record observation angle
                    # Get car's forward vector
                    npc_forward_vec = npc.get_transform().get_forward_vector()
                    
                    # Get magnitudes of car's forward vector and camera to obj vector
                    ray_m = ray.length()
                    npc_forward_vec_m = npc_forward_vec.length()
                    
                    # Calculate angle between vectors
                    alpha = acos(ray.dot(npc_forward_vec)/(npc_forward_vec_m*ray_m))

                    # Set observation angle
                    label.set_alpha(alpha)

                    f.write(label.__str__())  
    
    f.close()