import carla
import random
import yaml
import time

from core.cleanup import cleanup as clu

from actors.vehicle import Vehicle
from actors.rgbcam import RGBCam

class CarlaEnv:

    def load_carla_config(self, config_path="config/config.yaml"):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return config

    def connect_client(self, host='localhost', port=2000):
        client = carla.Client(host, port)
        client.set_timeout(5.0)
        return client
    
    def set_world_settings(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = self.config["simulation"]["synchronous_mode"]

    def __init__(self):
        self.config = self.load_carla_config()

        self.client = self.connect_client()
        self.client.load_world(self.config["world"])
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

        # to change vehicle change config bp id
        self.ego_vehicle = Vehicle(self.world, self.config["simulation"]["ego_vehicle_bp_id"])

        self.set_world_settings()

    def reset(self):
        self.collision_hist = []
        self.actor_list = []

        self.ego_vehicle.spawn()
        self.actor_list.append(self.ego_vehicle.actor)
        print(self.ego_vehicle.spawn_point)

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))

        # camera bp from config
        self.rgb_cam = RGBCam(self.world, self.config["simulation"]["rgb_camera_id"], host_actor=self.ego_vehicle.actor, spawn_transform=transform)
        self.rgb_cam.set_attribute("image_size_x", self.rgb_cam.IM_WIDTH)
        self.rgb_cam.set_attribute("image_size_y", self.rgb_cam.IM_HEIGHT)
        self.rgb_cam.spawn()
        self.actor_list.append(self.rgb_cam.actor)

        # tells camera sensor to start processing image data
        self.rgb_cam.start_listening()

        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        time.sleep(4)


        # TODO: maybe make a sensor class? have different sensor inherit from it
        col_sensor = self.blueprint_library.find(self.config["simulation"]["collision_sensor_id"])
        self.col_sensor = self.world.spawn_actor(col_sensor, transform, attach_to=self.ego_vehicle.actor)
        self.actor_list.append(self.col_sensor)

        # TODO: take action on collision
        self.col_sensor.listen(lambda event: self.collision_data(event))

    def cleanup(self):
        clu(self.client, self.actor_list)
        self.actor_list = []
        self.collision_hist = []
