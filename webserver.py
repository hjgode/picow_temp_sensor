import network
import socket
from time import sleep

from picozero import pico_led
import machine # only on raspberry pico w or m5core
import rp2

import sys
import io

# forked from: https://github.com/micropython/micropython-lib/tree/master/micropython/umqtt.robust
import utime
import simple

import ntptime

from machine import Pin, I2C
import SI7021

sdaPIN=machine.Pin(0)
sclPIN=machine.Pin(1)
i2c_bus = 0
addr = 0x40

i2c=machine.I2C(i2c_bus, sda=sdaPIN, scl=sclPIN, freq=400000)
si = SI7021.SI7021(i2c, addr)


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
MQTT_TOPIC_TIMESTAMP = 'pico/sensor1/timestamp'

MQTT_CLIENT_ID = b"raspberrypi_picow"

# Constants from the RP2040 datasheet
VREF = 3.3  # Reference voltage for the ADC
ADC_RESOLUTION = 65535  # 16-bit ADC resolution
TEMPERATURE_SLOPE = -1.721  # Slope for temperature conversion (datasheet)
TEMPERATURE_OFFSET = 27  # Temperature offset at 0.706V (datasheet)
# Initialize ADC for the temperature sensor (ADC4)
temp_sensor = machine.ADC(4)
MQTT_TOPIC_CPU_TEMPERATURE = 'pico/rp2040_1/temperature'
    
def readCPUtemp():
    # Read the raw ADC value
    raw_value = temp_sensor.read_u16()
    # Convert raw value to voltage
    voltage = (raw_value / ADC_RESOLUTION) * VREF
    # Calculate temperature in Celsius using the RP2040 formula
    temperature_celsius = TEMPERATURE_OFFSET - (voltage - 0.706) / TEMPERATURE_SLOPE
    return round(temperature_celsius)

def readTemp():
    temperature = si.temperature
    humidity = si.relative_humidity
    return round(temperature) ,round(humidity)

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

    def publish(self, topic, msg, retain=True, qos=1):
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

def publish_mqtt(topic, value):
    client.publish(topic, value, True, 1)
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
    print(f'connect(): Connected on {ip}')
    
    return ip

def get_sensor_readings():
    return 14, 1000, 60

def getTimeStr():
    rtc = machine.RTC()
    year, month, day, weekday, hour, minute, second, subsecond = rtc.datetime()
    print(f'Current time: {year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}')
    timeStr=f'{year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}'
    return timeStr

ip = connect()

try:
    # Connect to MQTT broker, start MQTT client
    client = connect_mqtt()
    while True:
        ntptime.settime()
        # Read sensor data
        #temperature, humidity, pressure = get_sensor_readings()
        temperature,humidity=readTemp()
        
        # Publish as MQTT payload
        publish_mqtt(MQTT_TOPIC_TEMPERATURE, str(temperature))
#        publish_mqtt(MQTT_TOPIC_PRESSURE, str(pressure))
        publish_mqtt(MQTT_TOPIC_HUMIDITY, str(humidity))

        cpuTemp=readCPUtemp()
        publish_mqtt( MQTT_TOPIC_CPU_TEMPERATURE, str(cpuTemp))
        
        timeStr=getTimeStr()
        publish_mqtt( MQTT_TOPIC_TIMESTAMP, timeStr)
        
        # Delay 10 seconds
        for i in range (0, 300, 1):
            # blink LED all 30 seconds
            if i % 30 == 0:
                pico_led.on()
                sleep(0.5)
                pico_led.off()
                sleep(0.5)
            else:
                sleep(1)
            if rp2.bootsel_button() == 1:
                sys.exit()

except Exception as e:
    print('Error:', e)
    sys.print_exception(e)