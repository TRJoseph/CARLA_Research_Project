import carla
import random
import numpy as np

from actors.vehicle import Vehicle
from custom_agents.ego_agent import EgoAgent
from kinematic_models.bicycle_model import BicycleModel
from controllers.MPC import MPCController


class EgoVehicle(Vehicle):
    def __init__(self, world, bp=None, spawn_point=None):
        super().__init__(world, bp, spawn_point)
        super().spawn()

        self.model_params = self.compute_model_params()

        self.model = BicycleModel(self.model_params)
        self.controller = MPCController(self.model)

        # different agents can impose different control schemas to the vehicle
        # the ego agent needs the vehicle actor and the controller to drive the vehicle
        self.agent = EgoAgent(self, self.controller, self.model)

    def get_current_state(self):
        transform = self.get_transform()
        vehicle_midpoint = transform.location

        velocity = self.actor.get_velocity()
        speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        yaw_angle = np.deg2rad(transform.rotation.yaw)

        return np.array([vehicle_midpoint.x, vehicle_midpoint.y, yaw_angle, speed_ms])


    def compute_model_params(self):
        self.physics = self.actor.get_physics_control()

        front_left_wheel = self.physics.wheels[0]
        front_right_wheel = self.physics.wheels[1]
        back_left_wheel = self.physics.wheels[2]
        back_right_wheel = self.physics.wheels[3]

        front_axle_midpoint = np.array([
        (front_left_wheel.position.x + front_right_wheel.position.x) / 2.0,
        (front_left_wheel.position.y + front_right_wheel.position.y) / 2.0,
        (front_left_wheel.position.z + front_right_wheel.position.z) / 2.0
        ]) / 100

        back_axle_midpoint = np.array([
        (back_left_wheel.position.x + back_right_wheel.position.x) / 2.0,
        (back_left_wheel.position.y + back_right_wheel.position.y) / 2.0,
        (back_left_wheel.position.z + back_right_wheel.position.z) / 2.0])/100
        
        vehicle_midpoint = self.get_transform().location
        vehicle_mid_pos = np.array([vehicle_midpoint.x, vehicle_midpoint.y, vehicle_midpoint.z])

        # this is the euclidean distance from the front axle to the back axle
        wheelbase = np.linalg.norm(front_axle_midpoint - back_axle_midpoint)
        L_f = np.linalg.norm(front_axle_midpoint - vehicle_mid_pos)
        L_r = np.linalg.norm(back_axle_midpoint - vehicle_mid_pos)
        
        model_params = {
            "wheelbase": wheelbase,
            "L_f": L_f,
            "L_r": L_r,
            "delta_lim": [-np.deg2rad(front_left_wheel.max_steer_angle),np.deg2rad(front_left_wheel.max_steer_angle)],
            "a_lim": [-10, 4],
            "dt": 0.05,
            "horizon": 10,
            # x is the initial state vector comprised of [x pos; y pos; yaw angle; velocity]
            "initial_state": np.array([vehicle_midpoint.x, vehicle_midpoint.y, np.deg2rad(self.get_transform().rotation.yaw), 0.1])
        }

        return model_params


