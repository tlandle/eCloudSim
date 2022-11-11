import logging
from datetime import datetime
LOG_FILENAME = datetime.now().strftime('log/logfile_%H_%M_%S_%d_%m_%Y.log')

for handler in logging.root.handlers[:]:
    logging.root.removeHandler(handler)

open(LOG_FILENAME,'a+') # creates log file if it doesn't exist

logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)  



