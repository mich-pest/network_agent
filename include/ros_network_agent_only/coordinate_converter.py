#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from geographic_msgs.msg import GeoPoint
from tuw_multi_robot_msgs.msg import Graph
import sys
import math
import networkx as nx
import osmnx as ox
from ros_network_agent_only import name_mapper as nm, params
import time
from itertools import count
# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc


class CoordinateConverter():
    """
    Class defined to wrap the usage of the geonav library, 
    including the angle offset between map and UTM frame.
    Indeed, in the geonav library the local map frame is 
    supposed to be aligned with UTM frame (UTM grid is aligned 
    such that +X is east, and +Y is north). 
    """

    def __init__(self, 
                 local_map_frame_lat=params.map_origin_lat, 
                 local_map_frame_lng=params.map_origin_lng, 
                 local_map_frame_angle_offset=params.map_origin_angle):
       
       # Latitude coordinate of the local map frame
        self._local_map_frame_lat = local_map_frame_lat 
        # Longitude coordinate of the local map frame
        self._local_map_frame_lng = local_map_frame_lng 
        # Offset angle between local map and UTM frame (received in deg but saved in rad)
        self._local_map_frame_angle_offset = local_map_frame_angle_offset * math.pi / 180

    def get_local_map_frame_latlng(self):
        return self._local_map_frame_lat, self._local_map_frame_lng

    def set_local_map_frame_angle_offset(self, angle):
        self._local_map_frame_angle_offset = angle * math.pi / 180

    def from_latlng_to_xy(self, lat, lng):
        # Convert a lat/lon to a local x/y
        utm_x, utm_y = gc.ll2xy(lat, lng, self._local_map_frame_lat, self._local_map_frame_lng)
        print("not rotated: " + str(utm_x) + ", " + str(utm_y))

        if not self._local_map_frame_angle_offset == 0.0:
            # If map and UTM frame not aligned rotate xy to be aligned with map frame
            map_x = math.cos(self._local_map_frame_angle_offset) * utm_x + math.sin(self._local_map_frame_angle_offset) * utm_y
            map_y = - math.sin(self._local_map_frame_angle_offset) * utm_x + math.cos(self._local_map_frame_angle_offset) * utm_y

            print("rotated: " + str(map_x) + ", " + str(map_y)) 
        else:
            # No misalignment: xy coordinate of local and UTM frame coincide
            map_x, map_y = utm_x, utm_y
        
        return map_x, map_y

    def from_xy_to_latlng(self, map_x, map_y):
        # Convert a local x/y to a lat/lon 
        if not self._local_map_frame_angle_offset == 0.0:
            # If map and UTM frame not aligned rotate xy again on UTM frame 
            utm_x = math.cos(self._local_map_frame_angle_offset) * map_x - math.sin(self._local_map_frame_angle_offset) * map_y
            utm_y = math.sin(self._local_map_frame_angle_offset) * map_x + math.cos(self._local_map_frame_angle_offset) * map_y
        else:
            # No misalignment: xy coordinate of local and UTM frame coincide
            utm_x = map_x
            utm_y = map_y

        lat, lng = gc.xy2ll(utm_x, utm_y, self._local_map_frame_lat, self._local_map_frame_lng)
        
        return lat, lng

    def find_angle(self, latlng2, latlng3):
        """
        Finds offset angle between 2 points considering the origin as observer location.
        Example: to have +45 deg:

                  . P3

           P1 .   . P2
        (map O)

        """
        x1, y1 = self.get_local_map_frame_latlng()
        x2, y2 = self.from_latlng_to_xy(*latlng2)
        x3, y3 = self.from_latlng_to_xy(*latlng3)
        result = (math.atan2(y3 - y1, x3 - x1) - math.atan2(y2 - y1, x2 - x1)) * 180 / math.pi
        print("ANGLE FOUND: " +str(result))
        return result 
