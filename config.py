import logging

general = dict(
    animation = True,
    logLevel = logging.DEBUG,
    logFile = "navigation.log",
)

logger = logging.getLogger('matplotlib')
logger.setLevel(level=logging.CRITICAL)
