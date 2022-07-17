import queue

# vehicle class; allows for easy vehicle spawning
class Vehicle:
    def __init__(self, world, vehicle_blueprint, spawn_point, traffic_manager):
        self.vehicle_blueprint = vehicle_blueprint
        self.spawn_point = spawn_point
        self.traffic_manager = traffic_manager
        self.is_driving = False

        self.vehicle = world.spawn_actor(self.vehicle_blueprint, self.spawn_point)

    def __call__(self):
        return self.vehicle

    @property
    def is_driving(self):
      return self.is_driving

    @is_driving.setter
    def is_driving(self, value):
        self.is_drivng = value

    def enable_driving(self):
        self.vehicle.set_autopilot(True, self.traffic_manager.get_port())
        self.is_driving = True
    
    def get_id(self):
        return self.vehicle.id

# spawn car with ability to add sensors
class EgoVehicle(Vehicle):
    def __init__(self, world, vehicle_blueprint, spawn_point, traffic_manager):
        super().__init__(world, vehicle_blueprint, spawn_point, traffic_manager)
        self.world = world
        self.sensor_queues = {}
        self.sensors = {}

    def add_sensor(self, sensor_name, sensor_blue_print, sensor_initial_transform):
        new_sensor = self.world.spawn_actor(sensor_blue_print, sensor_initial_transform, attach_to=self.vehicle)
        self.sensors[sensor_name] = new_sensor
        self.sensor_queues[sensor_name] = queue.Queue()

    def get_sensors(self):
        return self.sensors

    def enable_sensor_queue(self, sensor_name):
        self.sensors[sensor_name].listen(self.sensor_queues[sensor_name].put)
    
    def get_sensor_queue(self, sensor_name):
        return self.sensor_queues[sensor_name]