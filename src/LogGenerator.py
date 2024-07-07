import logging
import time

def LoggerGenerator(logfile = f"{time.time()}.log", loggername = "LogGenerator", consoleLevel = logging.WARNING):
        logger = logging.getLogger(loggername)
        logger.setLevel(logging.DEBUG)
        textformatter = logging.Formatter('%(asctime)s %(levelname)s %(filename)s: %(message)s')
        logformatter = logging.Formatter('%(created)f:%(levelname)s:%(filename)s;%(message)s')
        if logfile != "":
            fileHandler = logging.FileHandler(logfile, mode = "w")
            fileHandler.setLevel(logging.DEBUG)
            fileHandler.setFormatter(logformatter)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setLevel(consoleLevel)
        consoleHandler.setFormatter(textformatter)

        logger.addHandler(consoleHandler)
        if logfile != "":
            logger.addHandler(fileHandler)

        return logger


