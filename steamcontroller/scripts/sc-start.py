import os
from time import sleep
while(True):
    try:
        print "starting"
        os.system("sudo sc-xbox.py debug")
    except:
        pass
    print "restarting in 3s"
    sleep(3)
