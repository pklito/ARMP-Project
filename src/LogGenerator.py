import logging
import time

class LogGenerator:
    def __init__(self, logfile = f"{time.time()}.log", loggername = __name__, consoleLevel = logging.WARNING):
        logger = logging.getLogger(loggername)
        logger.setLevel(logging.DEBUG)
        textformatter = logging.Formatter('%(asctime)s ~ %(levelname)s ~ %(message)s')
        logformatter = logging.Formatter('%(created)f ~ %(levelname)s ~ %(message)s')
        if logfile != "":
            fileHandler = logging.FileHandler(logfile)
            fileHandler.setLevel(logging.DEBUG)
            fileHandler.setFormatter(logformatter)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setLevel(consoleLevel)
        consoleHandler.setFormatter(textformatter)

        logger.addHandler(consoleHandler)
        if logfile != "":
            logger.addHandler(fileHandler)

        self.logger = logger

    def log(self, data_dict, level = logging.INFO):
        if data_dict is not None:
            self.logger.log(level, str(data_dict))

    def debug(self, data):
        self.log(data, level=logging.DEBUG)

    def info(self, data):
        self.log(data, level=logging.INFO)

    def warning(self, data):
        self.log(data, level=logging.WARNING)

    def error(self, data):
        self.log(data, level=logging.ERROR)

    def critical(self, data):
        self.log(data, level=logging.CRITICAL)


