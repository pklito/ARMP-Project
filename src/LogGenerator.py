import logging
import time

class LogGenerator:
    def __init__(self, logfile = f"{time.time()}.log", loggername = __name__, consoleLevel = logging.WARNING):
        logger = logging.getLogger(loggername)
        logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(created)f ~ %(levelname)s ~ %(message)s')

        fileHandler = logging.FileHandler(logfile)
        fileHandler.setLevel(logging.INFO)
        fileHandler.setFormatter(formatter)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setLevel(consoleLevel)
        consoleHandler.setFormatter(formatter)

        logger.addHandler(consoleHandler)
        logger.addHandler(fileHandler)

        self.logger = logger

    def log(self, data_dict, level = logging.INFO):
        if data_dict is not None:
            self.logger.log(level, str(data_dict))

