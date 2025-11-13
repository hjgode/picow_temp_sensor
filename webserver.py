import network
import socket
from time import sleep

from picozero import pico_temp_sensor, pico_led
import machine # only on raspberry pico w or m5core
import rp2

import sys

# forked from: https://github.com/micropython/micropython-lib/tree/master/micropython/umqtt.robust
import utime
import simple

from machine import Pin, I2C
import SI7021

sdaPIN=machine.Pin(0)
sclPIN=machine.Pin(1)
i2c_bus = 0
addr = 0x40

i2c=machine.I2C(i2c_bus, sda=sdaPIN, scl=sclPIN, freq=400000)
si = SI7021.SI7021(i2c, addr)

def readTemp():
    temperature = si.temperature
    humidity = si.relative_humidity
    return round(temperature,ndigits=None) ,round(humidity,ndigits=None)

class MQTTClient(simple.MQTTClient):
    DELAY = 2
    DEBUG = False
    def delay(self, i):
        utime.sleep(self.DELAY)

    def log(self, in_reconnect, e):
        if self.DEBUG:
            if in_reconnect:
                print("mqtt reconnect: %r" % e)
            else:
                print("mqtt: %r" % e)

    def reconnect(self):
        i = 0
        while 1:
            try:
                return super().connect(False)
            except OSError as e:
                self.log(True, e)
                i += 1
                self.delay(i)

    def publish(self, topic, msg, retain=False, qos=0):
        while 1:
            try:
                return super().publish(topic, msg, retain, qos)
            except OSError as e:
                self.log(False, e)
            self.reconnect()

    def wait_msg(self):
        while 1:
            try:
                return super().wait_msg()
            except OSError as e:
                self.log(False, e)
            self.reconnect()

    def check_msg(self, attempts=2):
        while attempts:
            self.sock.setblocking(False)
            try:
                return super().wait_msg()
            except OSError as e:
                self.log(False, e)
            self.reconnect()
            attempts -= 1



ssid = 'Horst1'
password = '1234567890123'

# https://randomnerdtutorials.com/raspberry-pi-pico-w-mqtt-micropython/
MQTT_SERVER = b'192.168.0.40'
MQTT_PORT = 0
MQTT_USER = b''
MQTT_PASSWORD = b''
MQTT_KEEPALIVE = 7200

# Constants for MQTT Topics
MQTT_TOPIC_TEMPERATURE = 'pico/sensor1/temperature'
#MQTT_TOPIC_PRESSURE = 'pico/pressure'
MQTT_TOPIC_HUMIDITY = 'pico/sensor1/humidity'
MQTT_CLIENT_ID = b"raspberrypi_picow"

def connect_mqtt():
    try:
        client = MQTTClient(client_id=MQTT_CLIENT_ID,
                            server=MQTT_SERVER,
                            port=MQTT_PORT,
                            user=MQTT_USER,
                            password=MQTT_PASSWORD,
                            keepalive=MQTT_KEEPALIVE,
                )
        client.connect()
        return client
    except Exception as e:
        print('Error connecting to MQTT:', e)
        raise  # Re-raise the exception to see the full traceback

def publish_mqtt(topic, value, retain=True, qos=1):
    client.publish(topic, value, retain, qos)
    print(topic)
    print(value)
    print("Publish Done")


def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    
    while wlan.isconnected() == False:
        if rp2.bootsel_button() == 1:
            sys.exit()
        print('Waiting for connection...')
        pico_led.on()
        sleep(0.5)
        pico_led.off()
        sleep(0.5)
    
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    
    return ip

def get_sensor_readings():
    return 14, 1000, 60

ip = connect()


try:
    # Connect to MQTT broker, start MQTT client
    client = connect_mqtt()
    while True:
        # Read sensor data
        #temperature, humidity, pressure = get_sensor_readings()
        temperature,humidity=readTemp()
        
        # Publish as MQTT payload
        publish_mqtt(MQTT_TOPIC_TEMPERATURE, str(temperature), retain=True, qos=1)
#        publish_mqtt(MQTT_TOPIC_PRESSURE, str(pressure))
        publish_mqtt(MQTT_TOPIC_HUMIDITY, str(humidity), retain=True, qos=1)

        # Delay 10 seconds
        for i in range (0, 300, 1):
            sleep(1)
            if rp2.bootsel_button() == 1:
                sys.exit()

except Exception as e:
    print('Error:', e)
    