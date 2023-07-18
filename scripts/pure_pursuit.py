#! /usr/bin/env python3

import numpy as np

import rospy
from rospy import Publisher, Rate
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import math
import time
import osmnx as ox
from ros_network_agent_only import params
from ros_network_agent_only.coordinate_converter import CoordinateConverter
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.interfaces import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler, RVIZPathHandler
from ros_network_agent_only.network_agent import ROSNetworkAgent

from ros_network_agent_only.path_smoother import PathSmoother
from ros_network_agent_only.path_interface import PathInterface


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def assert_points(pts):
    assert isinstance(pts, (list, tuple)), 'points is of wrong type, expected list'
    for xy in pts:
        assert isinstance(xy, (list, tuple)), 'points contain an element of wrong type, expected list of two values (x, y)'
        assert len(xy), 'points contain an element of wrong type, expected list of two values (x, y)'
        x, y = xy
        assert isinstance(x, (int, float)), 'points contain a coordinate pair wherein one value is not a number'
        assert isinstance(y, (int, float)), 'points contain a coordinate pair wherein one value is not a number'

def publish_initialpose(state, n=10):

    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.pose.pose.position.x = state.x
    p.pose.pose.position.y = state.y

    q = quaternion_from_euler(0, 0, state.yaw)
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]

    pub = Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = Rate(10)

    for _ in range(n):
        pub.publish(p)
        rate.sleep()


class pure_pursuit:

    DELTA_TIME = 0.01
    TRAJ_LEN = 10
    TARGET_VELOCITY = 1.0
    RATE = 1e9
    TARGET_POINT = [3.0, 2.0]

    def __init__(self):

        ## Initialize network agent
        self._v = ROSNetworkAgent(vehicle_id=1)
        # Static copy of graph used to convert lat,lng -> x,y
        self.G = ox.io.load_graphml(params.graph_load_path) 
        self.cc = CoordinateConverter(
                local_map_frame_lat=params.map_origin_lat, 
                local_map_frame_lng=params.map_origin_lng, 
                local_map_frame_angle_offset=params.map_origin_angle)

        ## Parameters
        self.IS_SIM = load_param('~is_sim', False)
        self.USE_RVIZ = load_param('~use_rviz', False)
        self.STATE = load_param('~state', [0, 0, 0, 0])
        self.POINTS = []
        self.route_ids = []

        ## Set initial values for node

        # initial state
        state = VehicleState(*self.STATE)
        publish_initialpose(state)

        # Define a path to destination
        _, self.route_ids, _, _ = self._v.ask_for_path(self.cc.from_xy_to_latlng(state.x, state.y), 
                                                  self.cc.from_xy_to_latlng(*self.TARGET_POINT))
        for n in self.route_ids:
            x = round(float(self.G.nodes[n]['map_x']), 3)
            y = round(float(self.G.nodes[n]['map_y']), 3)
            self.POINTS.append([x,y])
        
        assert_points(self.POINTS)

        self.publish_global_path(smooth=False)

        # create goal state
        self.curr = 0
        self.goal = self.POINTS[self.curr]
        xs, ys = self.compute_traj(state)

        ## Create simulators, models, managers, etc.

        if self.IS_SIM:

            # simulator need a model to simulate
            self.sim_model = SimpleBicycleModel(state)

            # start the simulator immediately, but paused
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True).start()

        # start the SVEA manager
        self.svea = SVEAPurePursuit(LocalizationInterface,
                                    PurePursuitController,
                                    xs, ys,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)

        self.svea.controller.target_velocity = self.TARGET_VELOCITY
        self.initial_time_curr_edge = time.time()
        self.svea.start(wait=True)

        # everything ready to go -> unpause simulator
        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def publish_global_path(self, smooth=False):
        if smooth == True:
            # Add intermediate waypoints
            linsp_path_x = []
            linsp_path_y = [] 
            for idx, _ in enumerate(self.POINTS):
                if idx < len(self.POINTS)-1:
                    linsp_path_x = linsp_path_x + list(np.linspace(self.POINTS[idx][0], self.POINTS[idx+1][0], 10))
                    linsp_path_y = linsp_path_y + list(np.linspace(self.POINTS[idx][1], self.POINTS[idx+1][1], 10))
            self.POINTS = list(zip(linsp_path_x, linsp_path_y))
            
            # Smooth the path
            ps = PathSmoother(list(zip(linsp_path_x, linsp_path_y)))
            x, y, _, _ = ps.approximate_b_spline_path(degree=3, s=0.15)#(degree=4, s=0.05)
            self.POINTS = list(zip(x,y))

        print(self.POINTS)

        # Publish the path
        self.pi1 = PathInterface(self.POINTS, topic='/global_path') 

    def run(self):
        while self.keep_alive():

            self.pi1._points_path = self.POINTS
            self.pi1.create_pose_path()
            self.pi1.publish_rviz_path()

            # Check distance to upcoming node
            dist_to_next_node = math.dist([self.svea.state.x, self.svea.state.y], self.POINTS[self.curr])
            if dist_to_next_node < 0.2:

                # If svea is on last waypoint
                if self.curr == len(self.POINTS) - 1:
                    print("On target!")
                    rospy.sleep(100) # termination here
                
                # Publish data regarding the visited edge
                if self.curr - 1 >= 0:
                    value = round(time.time() - self.initial_time_curr_edge, 3) 
                    edge = [self.route_ids[self.curr - 1], self.route_ids[self.curr]]
                    self._v._send_update_edge_request(from_node=edge[0], to_node=edge[1], attr_name="travel_time", value=value)
                    print(f"Updating {edge[0]}, {edge[1]} with value {value}")

                # Ask for new path (replanning triggered)
                if self._v.replan_needed:
                    self.POINTS = []
                    _, self.route_ids, _, _ = self._v.ask_for_path(self.cc.from_xy_to_latlng(self.svea.state.x, self.svea.state.y), self.cc.from_xy_to_latlng(*self.TARGET_POINT))
                    for n in self.route_ids:
                        x = round(float(self.G.nodes[n]['map_x']), 3)
                        y = round(float(self.G.nodes[n]['map_y']), 3)
                        self.POINTS.append([x,y])
                    self.publish_global_path(smooth=False)
                    self.curr = 0
                    self.goal = self.POINTS[self.curr]

            self.spin()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):

        # limit the rate of main loop by waiting for state
        state = self.svea.wait_for_state()

        if self.svea.is_finished:
            self.update_goal()
            xs, ys = self.compute_traj(state)
            self.svea.update_traj(xs, ys)

        steering, velocity = self.svea.compute_control()
        self.svea.send_control(steering, velocity)

        self.svea.visualize_data()

    def update_goal(self):
        self.curr += 1
        self.curr %= len(self.POINTS)
        self.goal = self.POINTS[self.curr]
        self.svea.controller.is_finished = False
        self.initial_time_curr_edge = time.time()

    def compute_traj(self, state):
        xs = np.linspace(state.x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(state.y, self.goal[1], self.TRAJ_LEN)
        return xs, ys


if __name__ == '__main__':

    ## Start node ##

    pure_pursuit().run()