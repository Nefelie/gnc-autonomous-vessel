import time

import zeroros
from zeroros.messages import String

robot_ip = "192.168.10.1"
shutdown_pub = zeroros.Publisher("/shutdown", String, ip=robot_ip)
while True:
    shutdown_pub.publish(String(""))
    time.sleep(0.5)
