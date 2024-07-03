import src.LogGenerator
from time import time
a = src.LogGenerator.LogGenerator(logfile="test.log",consoleLevel=0)
a.log("start")

last_time = 0
while True:
    if time() - last_time > 0.3:
        a.info(time())
        last_time = time()
