#!/usr/bin/env python

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

        # Random route testing
        #current_pos = [random.uniform(params.south, params.north), random.uniform(params.west, params.east)]
        current_pos = [59.333974, 18.057215]
        #target_point = [random.uniform(params.south, params.north), random.uniform(params.west, params.east)]
        target_point = [59.336466, 18.062395]
        route, route_ids, time_s, consumption = v.ask_for_path(current_pos, target_point)
        v.print_route_stats(time_s, consumption, route=route)

        while not rospy.is_shutdown():
            if v.replan_needed:
                route, route_ids, time_s, consumption = v.ask_for_path(current_pos, target_point)
                v.print_route_stats(time_s, consumption, route=route)
            else:
                #v.set_location(move_along(route, route_idx))
                rospy.sleep(.3)


        #for i in range(6):
            #edge_idx = random.randint(0, len(params.example_edges) - 1)
            #edge = params.example_edges[edge_idx]
            
            #if i > 4:
            #    ttime = random.uniform(2.0, 10.0)
            #else:
            #    ttime = random.uniform(40.0, 70.0)
            ### Edge update testing

        #edge = (678831937, 1211118909)
        #v._send_update_edge_request(from_node=edge[0], to_node=edge[1], attr_name="n_vehicles", value=10)

if __name__ == '__main__':
    main()

        
