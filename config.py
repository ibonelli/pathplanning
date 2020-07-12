import logging

general = dict(
	animation = True,
	logLevel = logging.DEBUG,
	logFile = "navigation.log",
	grafDelay = 0.01,
	motion_model_limit = 3,
	lidar_steps = 8,
	wall_detection_threshold = 3,
)

logging.basicConfig(filename=general['logFile'],
					level=general['logLevel'],
					filemode = 'w')

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
