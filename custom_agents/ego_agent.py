import carla
import cvxpy

from agents.navigation.basic_agent import BasicAgent

from agents.navigation.global_route_planner import GlobalRoutePlanner

class EgoAgent(BasicAgent):
    def __init__(self, vehicle, target_speed=20, debug=False):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param target_speed: speed (in Km/h) at which the vehicle will move
        """

        super().__init__(vehicle, target_speed)


    def set_route(self, start_location, end_location):

        # Use global planner to get route
        route = self._global_planner.trace_route(start_location, end_location)
        
        # Feed route to local planner
        self._local_planner.set_global_plan(route)
    

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # TODO: set global plan??
        #test_path_plan = self._global_planner.trace_route(self._vehicle.Transform.location, 183????)

        target_waypoint = self._local_planner.get_incoming_waypoint_and_direction()[0]
        
        # Get upcoming waypoints for MPC reference
        waypoint_buffer = self._local_planner.get_plan()



        # Actions to take during each simulation step
        control = carla.VehicleControl()
        return control