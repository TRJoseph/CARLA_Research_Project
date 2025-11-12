import carla
import random
from actors.vehicle import Vehicle

def spawn_vehicles(world: carla.World, num=50, max_attempts=200) -> list[Vehicle]:
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in the current map.")

    vehicles = []
    attempts = 0

    while len(vehicles) < num and attempts < max_attempts:
        spawn_point = random.choice(spawn_points)
        vehicle = Vehicle(world, spawn_point=spawn_point)
        actor = vehicle.spawn()
        if actor is not None:
            vehicles.append(vehicle)
        attempts += 1

    print(f"[spawn_vehicles] Successfully spawned {len(vehicles)}/{num} vehicles after {attempts} attempts.")
    return vehicles