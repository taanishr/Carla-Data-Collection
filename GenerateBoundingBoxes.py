import numpy

# generate Bounding Boxes
class GenerateBoundingBoxes():
    def __init__(self, vehicle, projection_matrix, camera_matrix):
        self.vehicle = vehicle
        self.projection_matrix = projection_matrix
        self.camera_matrix = camera_matrix

        # list of pairs of vertex indexes on car bounding box that create an edge
        self.edge_pairs = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

    # build a projection matrix based on camera intrinsics
    @staticmethod
    def build_projection_matrix(width, height, fov=90):
        focal = width / (2.0 * numpy.tan(fov * numpy.pi / 360.0))
        projection_matrix = numpy.identity(3)
        projection_matrix[0, 0] = projection_matrix[1, 1] = focal
        projection_matrix[0, 2] = width / 2.0
        projection_matrix[1, 2] = height / 2.0
        return projection_matrix

    # convert 3d image pt to 2d image pt
    @staticmethod
    def get_image_point(location, projection_matrix, camera_matrix):
        # get coordinates from actual 3d point, then convert it to the camera's coordinates
        point = numpy.array([location.x, location.y, location.z, 1])
        # point changed to camera cords
        point_pc = numpy.dot(camera_matrix, point)

        # changing x y z system to y -z x system
        point_pc = [point_pc[1], -point_pc[2], point_pc[0]]

        # project 3d camera point to 2d using dot multiplication
        image_point = numpy.dot(projection_matrix, point_pc)

        # divide by z values
        image_point[1] = image_point[1] / image_point[2]
        image_point[0] = image_point[0] / image_point[2]

        return image_point[0:2]
    
    

    # builds a 3d bounding box
    def build3dBoundingBox(self):
        edges = []

        # get vertices from bounding box
        vertices = [v for v in self.vehicle.bounding_box.get_world_vertices(self.vehicle.get_transform())]

        # Get the vertices for each edge pair and append it to the list of edges
        for edge_pair in self.edge_pairs:
            p1 = GenerateBoundingBoxes.get_image_point(vertices[edge_pair[0]], self.projection_matrix, self.camera_matrix)
            p2 = GenerateBoundingBoxes.get_image_point(vertices[edge_pair[1]], self.projection_matrix, self.camera_matrix)
            edges.append([p1, p2])

        return edges
        

    # builds a 2d bounding box
    def build2dBoundingBox(self):
        vertices = [v for v in self.vehicle.bounding_box.get_world_vertices(self.vehicle.get_transform())]
        
        # set to extreme values for comparison with points
        x_max = -10000
        x_min = 10000
        y_max = -10000
        y_min = 10000

        # look for exterme vertices to create 2d bounding box
        for vertex in vertices:
            p = GenerateBoundingBoxes.get_image_point(vertex, self.projection_matrix, self.camera_matrix)
            # find the rightmost vertex
            if p[0] > x_max:
                x_max = p[0]
            # find the leftmost vertex
            if p[0] < x_min:
                x_min = p[0]
            # find the highest vertex
            if p[1] > y_max:
                y_max = p[1]
            # find the lowest  vertex
            if p[1] < y_min:
                y_min = p[1]
        
        # return points
        return x_max, x_min, y_max, y_min