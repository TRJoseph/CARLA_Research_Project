import random
import carla
import time
import yaml
from core.spawn_utils import *
from core.autopilot import *
from core.cleanup import *
from actors.vehicle import Vehicle
from actors.rgbcam import RGBCam
from carla_env import CarlaEnv

def main():
    c_env = CarlaEnv()


    try:
        # # TODO: set this to a specific spawn point
        # spawn_points = world.get_map().get_spawn_points()
        # spawn_location_transform = random.choice(spawn_points)

        # ego_vehicle = Vehicle(world, config["simulation"]["ego_vehicle_bp_id"], spawn_location_transform)
        # ego_vehicle.spawn()

        # if spawn_random:
        #     vehicles = spawn_vehicles(world, 50)

        # # if we want to use the rule based autopilot on the ego vehicle
        # if ego_vehicle_rule_based_ap:  
        #     vehicles.append(ego_vehicle)

        # if use_camera and ego_vehicle:
        #     camera = RGBCam(world, config["simulation"]["camera_type_id"], ego_vehicle.actor, carla.Transform(carla.Location(x=2.5, z=1.5)))
        #     camera.set_attribute("image_size_x", camera.IM_WIDTH)
        #     camera.set_attribute("image_size_y", camera.IM_HEIGHT)
        #     camera.spawn()

       
        # ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

        # toggle_autopilot(vehicles)
        # 
        
        #c_env.draw_world_spawn_points()

        c_env.reset()

        spectator = c_env.world.get_spectator()
        transform = c_env.ego_vehicle.spawn_point
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(pitch=-90)))

        print("Simulation running... Press Ctrl+C to stop.")

        while True:
            #time.sleep(0.1)
            c_env.step_forward()
    except KeyboardInterrupt:
        print("Stopping simulation...")
    finally:
        print("Cleaning up...")
        c_env.cleanup()
        c_env.step_forward()


if __name__ == '__main__':
    main()