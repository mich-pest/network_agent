#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
import random
from std_msgs.msg import Bool
from ros_network_agent_only import name_mapper as nm, params
from ros_network_agent_only.srv import *

class ROSNetworkAgent():

    def __init__(self, vehicle_id):
        if vehicle_id < 0:
            node_name = 'ROSNetworkAgent_{}'.format(abs(vehicle_id))
            topic_name = 'replan_needed_{}'.format(abs(vehicle_id))
        else:
            node_name = 'ROSNetworkAgent{}'.format(vehicle_id)
            topic_name = 'replan_needed{}'.format(vehicle_id)
        rospy.init_node(node_name)
        rospy.loginfo('{}: Alive!'.format(vehicle_id))
        self._id = int(vehicle_id)

        # Define subscriber to replan topic
        rospy.Subscriber(topic_name, Bool, self._handle_trigger)
        
        rospy.wait_for_service(nm.HANDSHAKE_SERVICE)
        self._handshake_service_client = rospy.ServiceProxy(nm.HANDSHAKE_SERVICE, 
                                                         Handshake)
        rospy.wait_for_service(nm.FIND_PATH_SERVICE)
        self._find_path_service_client = rospy.ServiceProxy(nm.FIND_PATH_SERVICE, 
                                                         FindPath)
        rospy.wait_for_service(nm.UPDATE_EDGE_SERVICE)
        self._update_edge_service_client = rospy.ServiceProxy(nm.UPDATE_EDGE_SERVICE, 
                                                         UpdateEdge)
        self._req_done = 0
        self._last_goal = None
        self._successive_failures = 0
        self._overall_failures = 0
        self.replan_needed = False
        self._location = None
        self._send_handshake_request()

    def _handle_trigger(self, msg):
        self.replan_needed = True

    def _send_handshake_request(self):
        handshake_request = HandshakeRequest()
        handshake_request.vehicle_id = self._id
        self._req_done += 1
        self._last_goal = handshake_request
        res = self._handshake_service_client(handshake_request)

        if(res.success == True):
            self._successive_failures = 0
            rospy.loginfo("{}: Hand-shake accepted (req #{}, {} failures, {} consecutive)".format(
                                self._id, self._req_done, self._overall_failures, self._successive_failures))
            return True
        else:
            
            rospy.loginfo("{}: Request NOT successful (req #{}, {} failures, {} consecutive), data: {}".format(
                        self._id, self._req_done, self._overall_failures, self._successive_failures, res.metadata.data))
            self._successive_failures += 1
            self._overall_failures += 1

            # If too many consecutive failures happened exception raised
            if self._successive_failures == params.max_successive_failure:
                raise rospy.ROSException("{} giving up, too many failures in a row (data: {})".format(self._id, res.metadata.data))

            # Metadata empty = server busy with others, retry for MAX_SUCCESSIVE_FAILURE-1 times max
            if res.metadata.data == '':
                rospy.sleep(random.randint(3, 20))
                self._send_handshake_request()
            # Fail, not due too many trials but due to something else, exception raised
            else:
                raise rospy.ROSException(res.metadata.data)

    def _send_find_path_request(self, source_gp=None, destination_gp=None):
        rospy.loginfo("{}: find path between ({},{}) and ({},{}) request"
                      .format(self._id, source_gp.latitude, source_gp.longitude, destination_gp.latitude, destination_gp.longitude))
        find_path_request = FindPathRequest()
        find_path_request.vehicle_id = self._id
        if None in (source_gp, destination_gp):
            raise rospy.ROSException("{}: source and/or destination not provided correctly, malformed request".format(self._id))
        else:
            find_path_request.client_location = source_gp 
            find_path_request.target_point = destination_gp
        self._req_done += 1
        self._last_goal = find_path_request
        res = self._find_path_service_client(find_path_request)

        if(res.success == True):
            self._successive_failures = 0
            rospy.loginfo("{}: Response retrieved (req #{}, {} failures, {} consecutive):".format(
                                self._id, self._req_done, self._overall_failures, self._successive_failures))

            return res
        else:
            rospy.loginfo("{}: Request NOT successful (req #{}, {} failures, {} consecutive), data: {}".format(
                            self._id, self._req_done, self._overall_failures, self._successive_failures, res.metadata.data))
            self._successive_failures += 1
            self._overall_failures += 1

            # If too many consecutive failures happened exception raised
            if self._successive_failures == params.max_successive_failure:
                raise rospy.ROSException("{} giving up, too many failures in a row (data: {})"
                                         .format(self._id, res.metadata.data))

            # Metadata empty = server busy with others, retry for MAX_SUCCESSIVE_FAILURE-1 times max
            if res.metadata.data == '':
                rospy.sleep(random.uniform(params.min_wait_time_failure, params.max_wait_time_failure))
                self._send_find_path_request(source_gp, destination_gp)
            # Fail, not due too many trials but due to something else, exception raised
            else:
                raise rospy.ROSException(res.metadata.data)

    def _send_update_edge_request(self, from_node, to_node, attr_name, value):
        rospy.loginfo("{}: Update ({}, {}) edge request ['{}': {}]"
                      .format(self._id, from_node, to_node, attr_name, value))
        edge_request = UpdateEdgeRequest()
        edge_request.vehicle_id = self._id
        edge_request.from_node = from_node
        edge_request.to_node = to_node
        s = String()
        s.data = attr_name
        edge_request.attr_name = s
        edge_request.value = value
        self._req_done += 1
        res = self._update_edge_service_client(edge_request)
        if(res.success == True):
            self._successive_failures = 0
            rospy.loginfo("{}: Data updated correctly (req #{})".format(self._id, self._req_done))
            return True
        else:
            rospy.loginfo("{}: Request NOT successful (req #{}, {} failures, {} consecutive), data: {}".format(
                            self._id, self._req_done, self._overall_failures, self._successive_failures, res.metadata.data))
            self._successive_failures += 1
            self._overall_failures += 1

            # If too many consecutive failures happened exception raised
            if self._successive_failures == params.max_successive_failure:
                raise rospy.ROSException("{} giving up, too many failures in a row (data: {})".format(self._id, res.metadata.data))

            # Metadata empty = server busy with others, retry for MAX_SUCCESSIVE_FAILURE-1 times max
            if res.metadata.data == '':
                rospy.sleep(random.randint(3, 20))
                self._send_update_edge_request(from_node, to_node, attr_name, value)
            # Fail, not due too many trials but due to something else, exception raised
            else:
                raise rospy.ROSException(res.metadata.data)

    def print_route_stats(self, time_s, consumption, route=None):
        rospy.loginfo("{}: travel time [s]: {}, consumption [%]: {}"
                      .format(self._id, time_s, consumption))
        #if route is not None:
        #    s = '['
        #    for n in route:
        #        s = s + '({}, {}), '.format(n.latitude, n.longitude)
        #    rospy.loginfo("{}: Obtained route: {}".format(self._id, s[0:-2] + ']'))
      
    def ask_for_path(self, source, destination):
        self.replan_needed = False
        source_gp = GeoPoint()
        destination_gp = GeoPoint()
        source_gp.latitude, source_gp.longitude = source
        destination_gp.latitude, destination_gp.longitude = destination
        res = self._send_find_path_request(source_gp=source_gp, 
                                           destination_gp=destination_gp)
        if res is None:
            while res is None:
                res = self._send_find_path_request(source_gp=source_gp, destination_gp=destination_gp)
        else:
            return res.route, res.route_ids, res.time_s, res.consumption

    def set_location(self, location):
        self._location = location

    def get_location(self):
        return self._location
    
    def perform_step(self, route, route_idx, data):
        pass
