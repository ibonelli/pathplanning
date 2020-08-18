import logging

general = dict(
	animation = True,
	logLevel = logging.DEBUG,
	logFile = "navigation.log",
	grafDelay = 0.01,
	motion_model_limit = 3,
	lidar_steps = 8,
	wall_detection_threshold = 3,
	grid_size = 1,       # potential grid size [m]
	# TODO - This should be flexible. Some functions are, others are not.
	# Now working with 5 for APF, BurshFire and Deliverative.
	robot_radius = 5.0,  # robot radius [m]
	vision_limit = 15,   # Lidar vision limit
	brushfire_radius_explore = 15,
	brushfire_radius_to_evaluate = 5,
	brushfire_neighbors_limit = 5,
	brushfire_map_debug = True,
	navData_debug = True,
	saveResults = True,
	robot_motion_model = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]],
	known_limit = 0.8,
	block_size = 4,
	trap_limit_distance = 5,
	astar_known_explore_range = 3,
	astar_known_limit = 0.2,
	show_Astar_animation = False,
)

logging.basicConfig(filename=general['logFile'],
					level=general['logLevel'],
					filemode = 'w')

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
