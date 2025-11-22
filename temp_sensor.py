import network
import socket
from time import sleep

USE_ESP32=True # use picoW or esp32?

if USE_ESP32:
    deviceType='ESP32'
else:
    deviceType='RP2040'
sensorName="sensor2"

import machine # only on raspberry pico w or m5core

if not USE_ESP32:
    from picozero import pico_led
    import rp2
else:
    import esp

import sys
import io
from machine import WDT

# forked from: https://github.com/micropython/micropython-lib/tree/master/micropython/umqtt.robust
import utime
from mqtt import MQTTClient
#import simple

import ntptime

if USE_ESP32:
    from machine import Pin, I2C
    import SI7021

    sdaPIN=machine.Pin(21)
    sclPIN=machine.Pin(22)
    i2c_bus = 0
    addr = 0x40

    i2c=machine.I2C(i2c_bus, sda=sdaPIN, scl=sclPIN, freq=400000)
    si = SI7021.SI7021(i2c, addr)
else:
    from machine import Pin
    from machine import Pin, I2C
    try:
        import SI7021

        sdaPIN=machine.Pin(21) #esp8266 Pin(4)
        sclPIN=machine.Pin(22) #esp8266 Pin(5)
        i2c_bus = 0
        addr = 0x40

        i2c=machine.I2C(i2c_bus, sda=sdaPIN, scl=sclPIN, freq=400000)
        si = SI7021.SI7021(i2c, addr)
    except Exception as e:
        print('Error:', e)
        sys.print_exception(e)

ssid = 'Horst1'
password = '1234567890123'

# https://randomnerdtutorials.com/raspberry-pi-pico-w-mqtt-micropython/
MQTT_SERVER = b'192.168.0.40'
MQTT_PORT = 0
MQTT_USER = b''
MQTT_PASSWORD = b''
MQTT_KEEPALIVE = 7200

# Constants for MQTT Topics
MQTT_TOPIC_TEMPERATURE = 'pico/' + sensorName + '/temperature'
#MQTT_TOPIC_PRESSURE = 'pico/pressure'
MQTT_TOPIC_HUMIDITY = 'pico/' + sensorName + '/humidity'
MQTT_TOPIC_TIMESTAMP = 'pico/' + sensorName + '/timestamp'
MQTT_TOPIC_IP = 'pico/' + sensorName + '/ip'
MQTT_TOPIC_DEVICE = 'pico/' + sensorName + '/device'

MQTT_CLIENT_ID = b"raspberrypi_picow"
      
if USE_ESP32:
    led = Pin(2, Pin.OUT)
    BUTTON_PIN = 0 #GPIO00
    button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    button_state = button.value()
    print("button value={button_state}")
    
def readCPUtemp():
    if not USE_ESP32:
        # Read the raw ADC value
        raw_value = temp_sensor.read_u16()
        # Convert raw value to voltage
        voltage = (raw_value / ADC_RESOLUTION) * VREF
        # Calculate temperature in Celsius using the RP2040 formula
        temperature_celsius = TEMPERATURE_OFFSET - (voltage - 0.706) / TEMPERATURE_SLOPE
        # Constants from the RP2040 datasheet
        VREF = 3.3  # Reference voltage for the ADC
        ADC_RESOLUTION = 65535  # 16-bit ADC resolution
        TEMPERATURE_SLOPE = -1.721  # Slope for temperature conversion (datasheet)
        TEMPERATURE_OFFSET = 27  # Temperature offset at 0.706V (datasheet)
        # Initialize ADC for the temperature sensor (ADC4)
        temp_sensor = machine.ADC(4)
        MQTT_TOPIC_CPU_TEMPERATURE = 'pico/' + sensorName + '/cpu_temperature'
        #endif
        return round(temperature_celsius)
    return 0

def readTemp():
    temperature = si.temperature
    humidity = si.relative_humidity
    return round(temperature) ,round(humidity)


def puback_cb(msg_id):
  print('PUBACK ID = %r' % msg_id)

def suback_cb(msg_id, qos):
  print('SUBACK ID = %r, Accepted QOS = %r' % (msg_id, qos))
  
def con_cb(connected):
  if connected:
    print("mqtt connected")
#    client.subscribe('subscribe/topic')

def msg_cb(topic, pay):
  print('Received %s: %s' % (topic.decode("utf-8"), pay.decode("utf-8")))


def connect_mqtt():
    try:
        client = MQTTClient(
                            server=MQTT_SERVER,
                            port=MQTT_PORT,
                            keep_alive=MQTT_KEEPALIVE
                )
        client.connect(client_id=MQTT_CLIENT_ID, user=MQTT_USER, password=MQTT_PASSWORD, will_qos=1, will_retain=True)
        return client
    except Exception as e:
        print('Error connecting to MQTT:', e)
        raise  # Re-raise the exception to see the full traceback

def publish_mqtt(topic, value):
  if client.isconnected():
    try:
        pub_id = client.publish('publish/topic', 'payload', False)
        print(topic, pub_id)
        print(value)
        print("Publish Done")
    except Exception as e:
      print(e)
    client.publish(topic, value, True, 1)

def blinkLED():
    if not USE_ESP32:
        pico_led.on()
    else:
        led.value(1)
    sleep(0.5)
    if not USE_ESP32:
        pico_led.off()
    else:
        led.value(2)
    sleep(0.5)

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    
    while wlan.isconnected() == False:
        if not USE_ESP32:
            if rp2.bootsel_button() == 1:
                sys.exit()
        else:
            if button.value == 1:
                sys.exit()
        print('Waiting for connection...')
        blinkLED()    
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

wdt = WDT(timeout=8000)  # enable watchdog with a timeout of 8s

try:
    client = MQTTClient(MQTT_SERVER, port=MQTT_PORT)
    client.set_connected_callback(con_cb)
    client.set_puback_callback(puback_cb)
    client.set_suback_callback(suback_cb)
    client.set_message_callback(msg_cb)
except Exception as e:
    print('MQTT Client Error:', e)
    sys.print_exception(e)
    try:
        with open('error.log', 'a') as f:
            import io
            sys.print_exception(e,f)
            
    except Exception as e:
        print(f"Failed to log exception: {e}")
   
try:
    publish_mqtt(MQTT_TOPIC_DEVICE, deviceType)
    publish_mqtt(MQTT_TOPIC_IP, str(ip))
    # Connect to MQTT broker, start MQTT client
    while True:
#        ntptime.settime()
        # Read sensor data
        #temperature, humidity, pressure = get_sensor_readings()
        temperature,humidity=readTemp()
        
        # Publish as MQTT payload
        publish_mqtt(MQTT_TOPIC_TEMPERATURE, str(temperature))
#        publish_mqtt(MQTT_TOPIC_PRESSURE, str(pressure))
        publish_mqtt(MQTT_TOPIC_HUMIDITY, str(humidity))

        if not USE_ESP32:
            cpuTemp=readCPUtemp()
            publish_mqtt( MQTT_TOPIC_CPU_TEMPERATURE, str(cpuTemp))
        
#        timeStr=getTimeStr()
#        publish_mqtt( MQTT_TOPIC_TIMESTAMP, timeStr)
        
        # Delay 10 seconds
        for i in range (0, 300, 1):
            # blink LED all 30 seconds
            if i % 30 == 0:
                blinkLED()
            else:
                sleep(1)
            wdt.feed() #reset watchdog timer
            if not USE_ESP32:
                #ifndef TEST
                if rp2.bootsel_button() == 1:
                    sys.exit()
            else:
                if button.value == 1:
                    sys.exit()

except Exception as e:
    print('Error:', e)
    sys.print_exception(e)
    try:
        with open('error.log', 'a') as f:
            import io
            sys.print_exception(e,f)
            
    except Exception as e:
        print(f"Failed to log exception: {e}")
