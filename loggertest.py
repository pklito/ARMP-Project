import src.LogGenerator
from time import time
a = src.LogGenerator.LoggerGenerator(logfile="test.log",consoleLevel=0)
a.debug("start")

last_time = 0
while True:
    if time() - last_time > 0.3:
        a.info(time())
        last_time = time()
