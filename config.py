import logging

general = dict(
	animation = True,
	logLevel = logging.DEBUG,
	logFile = "navigation.log",
	grafDelay = 0.01,
	motion_model_limit = 3
)

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
