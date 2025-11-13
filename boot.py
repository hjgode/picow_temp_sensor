# This file is executed on every boot (including wake-boot from deepsleep)
import gc
import webrepl
import config # SSID etc. importieren
from time import sleep
import rp2
import sys

def do_connect():
    import network
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(config.WLANSSID, config.WLANPW)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

do_connect()
webrepl.start()
gc.collect()

autorun=True
print("Press BootSel Button witin 10 Seconds to stop autostart")
for i in range(0,10,1):
    sleep(1)
    print (".")
    if rp2.bootsel_button() == 1:
#        sys.exit()
        autorun=False
        break
if autorun:        
    import webserver
else:
    print ("no autorun")