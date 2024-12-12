import time

import zeroros
from zeroros.messages import String

robot_ip = "192.168.10.1"
reboot_pub = zeroros.Publisher("/reboot", String, ip=robot_ip)
while True:
    reboot_pub.publish(String(""))
    time.sleep(0.5)
