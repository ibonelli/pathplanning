import logging

general = dict(
	animation = True,
	logLevel = logging.DEBUG,
	logFile = "navigation.log",
	grafDelay = 0.01,
	motion_model_limit = 3,
	lidar_steps = 8,
)

#LOG_FORMAT = '%(asctime)-15s %(clientip)s %(user)-8s %(message)s'
#logging.basicConfig(filename = program.log,
#					level=logging.CRITICAL,
#					format = LOG_FORMAT,
#					filemode = 'w')

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
