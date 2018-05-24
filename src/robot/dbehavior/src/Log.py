import logging
from Blackboard import getbb

logger = logging.getLogger('behavior')
logger.setLevel(logging.DEBUG)

logFolder = './log'
fileHandler = logging.FileHandler(logFolder)
logger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
logger.addHandler(consoleHandler)

bb = getbb()
ENABLE_LOG = bb.parameters.enableLog

if ENABLE_LOG:
    def debug(*args, **kwargs):
        logger.debug(*args, **kwargs)


    def info(*args, **kwargs):
        logger.info(*args, **kwargs)


    def warning(*args, **kwargs):
        logger.warning(*args, **kwargs)


    def error(*args, **kwargs):
        logger.error(*args, **kwargs)


    def critical(*args, **kwargs):
        logger.critical(*args, **kwargs)

else:
    def debug(*args, **kwargs):
        pass


    def info(*args, **kwargs):
        logger.info(*args, **kwargs)


    def warning(*args, **kwargs):
        logger.warning(*args, **kwargs)


    def error(*args, **kwargs):
        logger.error(*args, **kwargs)


    def critical(*args, **kwargs):
        logger.critical(*args, **kwargs)

