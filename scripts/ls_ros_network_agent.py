#!/usr/bin/env python

"""
Example of usage of ROSNetworkAgent instances in nodes executing at runtime. 
The agent provides some random points as starting and destination position and repeats the request if needed. 
Possibly, in the main loop it asks for updates related to the visited edges (e.g. travel_time).    
"""

import rospy
import sys
import random
from ros_network_agent_only import params
from ros_network_agent_only.network_agent import ROSNetworkAgent

def main():
    if len(sys.argv) >= 2:
        vehicle_id = int(sys.argv[1])
        v = ROSNetworkAgent(vehicle_id)
        rospy.sleep(random.randint(0, 5))

        # Pick random point
        current_pos = [random.uniform(params.south, params.north), random.uniform(params.west, params.east)]
        target_point = [random.uniform(params.south, params.north), random.uniform(params.west, params.east)]
        route, route_ids, time_s, consumption = v.ask_for_path(current_pos, target_point)
        v.print_route_stats(time_s, consumption, route=route)
        
        while not rospy.is_shutdown():
            # If replan is triggered, request new path
            if v.replan_needed:
                route, route_ids, time_s, consumption = v.ask_for_path(current_pos, target_point)
                v.print_route_stats(time_s, consumption, route=route)
            else:
                #v._send_update_edge_request(from_node=edge[0], to_node=edge[1], attr_name="n_vehicles", value=10)
                rospy.sleep(.3)

if __name__ == '__main__':
    main()

        
