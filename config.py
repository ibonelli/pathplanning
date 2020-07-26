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
	robot_radius = 5.0,  # robot radius [m]
	vision_limit = 15,   # Lidar vision limit
	brushfire_radius_explore = 15,
	brushfire_radius_to_evaluate = 4,
	robot_motion_model = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]],
)

logging.basicConfig(filename=general['logFile'],
					level=general['logLevel'],
					filemode = 'w')

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
