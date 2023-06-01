
####################################
# Colors 

WARNING_COLOR = '\33[330m'
YELLOW = '\033[1;33;48m'
STANDARD_COLOR = '\33[0m'
STATE_COLOR = '\33[7m'
LOW_BATTERY_COLOR = '\33[41m'

#####################################
# Graph creation
 
# ------ Graph by point
# Desired source point for the graph 
graph_origin = (59.337699, 18.059285) 
# KTH: (59.350989, 18.068410) 
# Taipei: (25.054736, 121.539063)

# Desired distance for the graph
dist = 1500

# Graph by bbox
small_park_coord = 59.341490, 59.338905, 18.075316, 18.070808 # already provided scenario (small_park)
medium_stock_coord = 59.342422, 59.332726, 18.068155, 18.048869 # already provided scenario (medium_stock)
small_stock_coord = 59.342324, 59.332526, 18.067241, 18.051436 
north, south, east, west = small_stock_coord

# Types of network represented in the graph
network_type = 'walk' #['walk', 'bike']
# Undesired tags in the graph
undesired_tags = set({'unclassified', 'steps', 'tertiary', 'elevator', 'secondary', 'primary', 'residential'})
# Tags included in the graph with a problematic meaning (assumptions needed)
warning_tags = set({'bridleway', 'service', 'track'})
# Attributes keys for which the static value is 0 (not needed to encode that in the edge)
zero_static = set({'n_vehicles', 'n_pedestrians'})
# Edge types to use for correctly loading the graph from file
edge_types = {
    "bearing": float,
    "grade": float,
    "grade_abs": float,
    "length": float,
    "osmid": int,
    "speed_kph": float,
    "travel_time": float,
    "travel_time_static": float,
    "battery_consumption": float,
    "battery_consumption_static": float,
    "road_risk": float,
    "road_risk_static": float,
    "n_vehicles": int,
    "n_pedestrians": int,
    "map_x": float,
    "map_y": float,
}

# Attributes actually on which perform decay
decay_attr_names = ['battery_consumption', 
                    'travel_time', 
                    'road_risk',
                    'n_pedestrians', 
                    'n_vehicles']

####################################
# Network management

dynamic_attributes = True
decay_time = 5.0  # Decay time for edges' attributes
monitor_time = 7.0
min_wait_time_failure = 0.1
max_wait_time_failure = 6.0
max_successive_failure = 10 # Number of maximum ack failures for the agent

# Fake agent data
example_edges = [(159940, 505617319), (159942, 1741721666), (159942, 2507550706), (159948, 1362749780), (159949, 1018920290), (159949, 1362749780), (159949, 1734187122), (159949, 1734187108), (159954, 1018920273), (159954, 1362749788), (159954, 6189537826), (159954, 1734187168), (8095622, 678831937), (8095622, 1014440996), (8095622, 1014440986), (8095622, 1211118813), (410522059, 442882340), (410522059, 442882342), (442882323, 1211118658), (442882323, 442882341), (442882323, 442882340), (442882340, 410522059), (442882340, 442882342), (442882340, 442882323), (442882341, 442882323), (442882341, 1211118495), (442882341, 1211118909), (442882341, 442882344), (442882342, 410522059), (442882342, 442882340), (442882342, 442882344), (442882344, 442882341), (442882344, 442882342), (442882344, 1741721683), (505617319, 864853608), (505617319, 159940), (505617319, 2507550706), (678831937, 8095622), (678831937, 1211118909), (678831937, 1211118813), (864853608, 505617319), (864853608, 1014440986), (1014440986, 1014440996), (1014440986, 8095622), (1014440986, 864853608), (1014440996, 8095622), (1014440996, 1741721680), (1014440996, 1014440986), (1018920273, 159954), (1018920273, 1734187169), (1018920273, 1734187175), (1018920290, 159949), (1018920290, 1734187123), (1018920290, 1734187126), (1018920384, 1362749788), (1211118495, 442882341), (1211118495, 1211118974), (1211118495, 1734187081), (1211118658, 442882323), (1211118658, 2507550706), (1211118658, 1211118813), (1211118733, 1362749780), (1211118733, 1211118974), (1211118733, 1734187108), (1211118813, 1211118658), (1211118813, 8095622), (1211118813, 678831937), (1211118909, 1741721680), (1211118909, 442882341), (1211118909, 1734187128), (1211118909, 678831937), (1211118974, 1211118495), (1211118974, 1211118733), (1362749780, 159949), (1362749780, 159948), (1362749780, 1734187102), (1362749780, 1211118733), (1362749788, 159954), (1362749788, 1018920384), (1362749788, 1734187152), (1362749788, 6189537825), (1525463203, 1734187128), (1734187081, 1211118495), (1734187081, 1734187128), (1734187081, 1734187102), (1734187102, 6189537825), (1734187102, 1362749780), (1734187102, 1734187081), (1734187102, 1734187122), (1734187108, 1734187120), (1734187108, 159949), (1734187108, 1211118733), (1734187120, 1734187108), (1734187120, 1734187123), (1734187122, 1734187124), (1734187122, 6189537826), (1734187122, 159949), (1734187122, 1734187102), (1734187123, 10240207165), (1734187123, 1734187120), (1734187123, 1018920290), (1734187124, 1734187126), (1734187124, 1734187166), (1734187124, 1734187122), (1734187126, 1734187124), (1734187126, 1018920290), (1734187128, 1734187081), (1734187128, 1211118909), (1734187128, 1734187152), (1734187128, 1525463203), (1734187152, 1734187168), (1734187152, 1734187128), (1734187152, 1362749788), (1734187166, 1734187124), (1734187166, 1734187169), (1734187166, 6189537826), (1734187168, 159954), (1734187168, 1734187173), (1734187168, 1734187152), (1734187169, 1734187166), (1734187169, 1018920273), (1734187173, 1734187175), (1734187173, 1734187168), (1734187175, 1018920273), (1734187175, 1734187173), (1741721666, 159942), (1741721673, 1741721680), (1741721680, 1211118909), (1741721680, 1014440996), (1741721680, 1741721673), (1741721683, 442882344), (2507550706, 1211118658), (2507550706, 7635188614), (2507550706, 505617319), (2507550706, 159942), (6189537825, 1734187102), (6189537825, 1362749788), (6189537825, 6189537826), (6189537826, 159954), (6189537826, 1734187122), (6189537826, 1734187166), (6189537826, 6189537825), (7635188613, 7635188614), (7635188613, 7635188614), (7635188614, 2507550706), (7635188614, 7635188613), (7635188614, 7635188613), (10240207165, 1734187123)]

# Data plot
edge_to_plot = (678831937, 1211118909)
attr_to_plot = "n_vehicles"

n_simulated_vehicles = 1 # Set this to 0 if using `spawn_agents.launch`
n_simulated_pedestrians = 15


attr_max_change_perc = 0.4
ped_detector_id = -1
pedestrian_done_topic = '/ped_done'
ped_start = 59.336386, 59.332687, 18.067964, 18.058863
ped_start_north, ped_start_south, ped_start_east, ped_start_west = ped_start
ped_target = 59.340604, 59.338695, 18.065866, 18.054744
ped_target_north, ped_target_south, ped_target_east, ped_target_west = ped_target

####################################
# Filepaths

# Path to use to load the graph file (test graph)
graph_load_path = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/graphml/small_stock.graphml"
# Path to use to save html interactive map
graph_save_interactive_map = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/html/example_graph.html"
graph_save_interactive_map_w_heatmap = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/html/example_graph_heatmap.html"
# Path to use to save graphml file
graph_save_graphml = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/graphml/filled_graph.graphml"
# Url template used to perform elevation API calls
graph_saved_from_grid = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/graphml/filled_graph_from_grid.graphml"
local_url_template = "http://localhost:5000/v1/eudem25m?locations={}&key={}" 
# OpenTopoData API link 
opentopo_url_template = "https://api.opentopodata.org/v1/eudem25m?locations={}&key={}"


####################################
# Graph format adapter

grid_graph_topic = "/segments"
output_grid_graph = "/home/michele/ros_workspace_network/src/ros_network_pkg/data/graphml/converted_grid_graph.graphml"


####################################
# Battery Modelling 

autonomy_minutes = 60
consumption_rate = 100 / (autonomy_minutes * 60) # [%/s]

MAX_SPEED = 1.7
NORMAL_SPEED = 1.0
