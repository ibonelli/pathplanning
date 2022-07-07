import logging

general = dict(
	animation = True, # Build animation
	showGrafs = True, # Show animation
	logLevel = logging.DEBUG,
	logFile = "navigation.log",
	grafDelay = 0.01,
	# Debe ser multiplo de 8
	lidar_steps = 8, # Used for initial work
	#lidar_steps = 32, # Decided as best option in most scenarios
	wall_detection_threshold = 3,
	grid_size = 1,       # potential grid size [m]
	robot_radius = 2.0,  # robot radius [m]
	#robot_radius = 5.0,  # robot radius [m]
	vision_limit = 10,   # Lidar vision limit used for initial work
	#vision_limit = 30,   # Lidar vision limit decided as best option in most scenarios
	brushfire_radius_explore = 15,
	brushfire_radius_to_evaluate = 5,
	brushfire_neighbors_limit = 5,
	brushfire_map_debug = True,
	navData_debug = True,
	saveResults = True,
	saveReport = True,
	robot_motion_model = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]],
	#  [-1, 1]   [0, 1]    [1, 1]
	#          \    |    /
	#  [-1, 0] -----+----- [1, 0]
	#          /    |    \
	# [-1, -1]   [0, -1]   [1, -1]
	known_limit = 0.8,
	block_size = 4,
	trap_limit_distance = 7,
	astar_known_explore_range = 3,
	astar_known_limit = 0.2,
	show_Astar_animation = False,
)

logging.basicConfig(filename=general['logFile'],
					level=general['logLevel'],
					filemode = 'w')

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
