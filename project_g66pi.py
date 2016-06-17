#!/usr/bin/env python3

# Send sample data periodically to AWS IoT.  Based on Grove sensors.

import time
import datetime
import ssl
import json
import paho.mqtt.client as mqtt
import grovepi
import smbus
from time import sleep

# TODO: Change this to the name of our Raspberry Pi, also known as our "Thing Name"
deviceName = "g66pi"


# Public certificate of our Raspberry Pi, as provided by AWS IoT.
deviceCertificate = "tp-iot-certificate.pem.crt"
# Private key of our Raspberry Pi, as provided by AWS IoT.
devicePrivateKey = "tp-iot-private.pem.key"
# Root certificate to authenticate AWS IoT when we connect to their server.
awsCert = "aws-iot-rootCA.crt"

isConnected = False

# Assume we connected the Grove Light Sensor to analog port A0,
# Digital Humidity/Temperature Sensor (DHT11) to digital port D2,
# Sound Sensor to A2, Grove LED to digital port D4.
# If you are using the Grove Analog Temperature Sensor, connect it to analog port A1.
light_sensor = 0
sound_sensor = 2
ultrasonic_ranger = 2
led = 4
temp_sensor = 1
buzzer = 8
# select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-2] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)
# ADXL345 constants
EARTH_GRAVITY_MS2   = 9.80665
SCALE_MULTIPLIER    = 0.004

DATA_FORMAT         = 0x31
BW_RATE             = 0x2C
POWER_CTL           = 0x2D

BW_RATE_1600HZ      = 0x0F
BW_RATE_800HZ       = 0x0E
BW_RATE_400HZ       = 0x0D
BW_RATE_200HZ       = 0x0C
BW_RATE_100HZ       = 0x0B
BW_RATE_50HZ        = 0x0A
BW_RATE_25HZ        = 0x09

RANGE_2G            = 0x00
RANGE_4G            = 0x01
RANGE_8G            = 0x02
RANGE_16G           = 0x03

MEASURE             = 0x08
AXES_DATA           = 0x32

class ADXL345:

    address = None

    def __init__(self, address = 0x53):        
        self.address = address
        self.setBandwidthRate(BW_RATE_100HZ)
        self.setRange(RANGE_2G)
        self.enableMeasurement()

    def enableMeasurement(self):
        bus.write_byte_data(self.address, POWER_CTL, MEASURE)

    def setBandwidthRate(self, rate_flag):
        bus.write_byte_data(self.address, BW_RATE, rate_flag)

    # set the measurement range for 10-bit readings
    def setRange(self, range_flag):
        value = bus.read_byte_data(self.address, DATA_FORMAT)

        value &= ~0x0F;
        value |= range_flag;  
        value |= 0x08;

        bus.write_byte_data(self.address, DATA_FORMAT, value)
    
    # returns the current reading from the sensor for each axis
    #
    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs
    def getAxes(self, gforce = False):
        bytes = bus.read_i2c_block_data(self.address, AXES_DATA, 6)
        
        x = bytes[0] | (bytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = bytes[2] | (bytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = bytes[4] | (bytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)

        x = x * SCALE_MULTIPLIER 
        y = y * SCALE_MULTIPLIER
        z = z * SCALE_MULTIPLIER

        if gforce == False:
            x = x * EARTH_GRAVITY_MS2
            y = y * EARTH_GRAVITY_MS2
            z = z * EARTH_GRAVITY_MS2

        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)

        return {"x": x, "y": y, "z": z}

if __name__ == "__main__":
    # if run directly we'll just create an instance of the class and output 
    # the current readings
    adxl345 = ADXL345()

    
    
# This is the main logic of the program.  We connect to AWS IoT via MQTT, send sensor data periodically to AWS IoT,
# and handle any actuation commands received from AWS IoT.
def main():
    global isConnected
    # Create an MQTT client for connecting to AWS IoT via MQTT.
    client = mqtt.Client(deviceName + "_sr")  # Client ID must be unique because AWS will disconnect any duplicates.
    client.on_connect = on_connect  # When connected, call on_connect.
    client.on_message = on_message  # When message received, call on_message.
    client.on_log = on_log  # When logging debug messages, call on_log.

    # Set the certificates and private key for connecting to AWS IoT.  TLS 1.2 is mandatory for AWS IoT and is supported
    # only in Python 3.4 and later, compiled with OpenSSL 1.0.1 and later.
    client.tls_set(awsCert, deviceCertificate, devicePrivateKey, ssl.CERT_REQUIRED, ssl.PROTOCOL_TLSv1_2)

    # Connect to AWS IoT server.  Use AWS command line "aws iot describe-endpoint" to get the address.
    print("Connecting to AWS IoT...")
    client.connect("A1P01IYM2DOZA0.iot.us-west-2.amazonaws.com", 8883, 60)

    # Start a background thread to process the MQTT network commands concurrently, including auto-reconnection.
    client.loop_start()

    # Configure the Grove LED port for output.
    grovepi.pinMode(led, "OUTPUT")
    time.sleep(1)

    # Loop forever.
    while True:
        try:
            # If we are not connected yet to AWS IoT, wait 1 second and try again.
            if not isConnected:
                time.sleep(1)
                continue
            # read temperature
            temp = grovepi.temp(temp_sensor,'1.1')
            distance_in_cm = grovepi.ultrasonicRead(ultrasonic_ranger)
           

            if distance_in_cm > 3 and distance_in_cm < 7 :
               grovepi.digitalWrite(led,1)  
               grovepi.digitalWrite(buzzer,0)
               msg = "Distance between bike and ground is more than 20 cm  ->Led on} : value of distance is %d " %distance_in_cm
            elif  distance_in_cm >= 7 :
               grovepi.digitalWrite(led, 1)
               grovepi.digitalWrite(buzzer,1)
               msg = "To much gap between bike and ground :->led on/buzzer on : value of distance is %d" %distance_in_cm
            else: 
               grovepi.digitalWrite(led,0)  
               grovepi.digitalWrite(buzzer,0)
               msg = "OK : distance is less than 20 cm: value of distance is %d" %distance_in_cm 

        #except KeyboardInterrupt:
        #    break
        #except IOError:
        #    print("error");
 
            #Get light sensor(LDR)/Sound_sensor value
            light_sensor_value = grovepi.analogRead(light_sensor)
            loudness_sensor_value = grovepi.analogRead(sound_sensor)
            axes = adxl345.getAxes(True)
            
            #time.sleep(1)
            # led off - buzzer off
            #grovepi.digitalWrite(led,0) 
            #grovepi.digitalWrite(buzzer,0)
            #time.sleep(1)
        
               # print ("temp =", temp)
                # Prepare our sample data in JSON format.
            payload = {
                "state": {
                    "reported": {
                        "temperature": temp,
                        "humidity": 55,
                        "light_level": light_sensor_value,
                        "sound_level": loudness_sensor_value,
                        "message": msg,
                        "distance":distance_in_cm,
                        "accelerometer":axes, 
                        "timestamp": datetime.datetime.now().isoformat()
                    }
                }
            }
            print("Sending sensor data to AWS IoT...\n" +
                  json.dumps(payload, indent=4, separators=(',', ': ')))

            # Publish our sensor data to AWS IoT via the MQTT topic, also known as updating our "Thing Shadow".
            client.publish("$aws/things/" + deviceName + "/shadow/update", json.dumps(payload))
            print("Sent to AWS IoT")

            # Wait 30 seconds before sending the next set of sensor data.
            time.sleep(0.5)

        except KeyboardInterrupt:
            # Stop the program when we press Ctrl-C.
            break
        except IOError:
            # Some I/O problem happened.
            print("I/O Error")
            continue
        except:
            # For all other errors, we wait a while and resume.
            time.sleep(10)
            continue


# This is called when we are connected to AWS IoT via MQTT.
# We subscribe for notifications of desired state updates.
def on_connect(client, userdata, flags, rc):
    global isConnected
    isConnected = True
    print("Connected to AWS IoT")
    # Subscribe to our MQTT topic so that we will receive notifications of updates.
    topic = "$aws/things/" + deviceName + "/shadow/update/accepted"
    print("Subscribing to MQTT topic " + topic)
    client.subscribe(topic)


# This is called when we receive a subscription notification from AWS IoT.
def on_message(client, userdata, msg):
    # Convert the JSON payload to a Python dictionary.
    # The payload is in binary format so we need to decode as UTF-8.
    payload2 = json.loads(msg.payload.decode("utf-8"))
    print("Received message, topic: " + msg.topic + ", payload:\n" +
          json.dumps(payload2, indent=4, separators=(',', ': ')))

    # If there is a desired state in this message, then we actuate, e.g. if we see "led=on", we switch on the LED.
    if payload2.get("state") is not None and payload2["state"].get("desired") is not None:
        # Get the desired state and loop through all attributes inside.
        desired_state = payload2["state"]["desired"]
        for attribute in desired_state:
            # We handle the attribute and desired value by actuating.
            value = desired_state.get(attribute)
            actuate(client, attribute, value)


# Control my actuators based on the specified attribute and value, e.g. "led=on" will switch on my LED.
def actuate(client, attribute, value):
    if attribute == "timestamp":
        # Ignore the timestamp attribute, it's only for info.
        return
    print("Setting " + attribute + " to " + value + "...")
    if attribute == "led":
        # We actuate the LED for "on", "off" or "flash1".
        if value == "on":
            # Switch on LED.
            grovepi.digitalWrite(led, 1)
            send_reported_state(client, "led", "on")
            return
        elif value == "off":
            # Switch off LED.
            grovepi.digitalWrite(led, 0)
            send_reported_state(client, "led", "off")
            return
        elif value == "flash1":
            # Switch on LED, wait 1 second, switch it off.
            grovepi.digitalWrite(led, 1)
            send_reported_state(client, "led", "on")
            time.sleep(1)

            grovepi.digitalWrite(led, 0)
            send_reported_state(client, "led", "off")
            time.sleep(1)
            return
    if attribute == "buzzer":
         if value == "on":
            grovepi.digitalWrite(buzzer,1)
            send_reported_state(client, "buzzer", "on")
            return
         if value == "off":
            grovepi.digitalWrite(buzzer,0)
            send_reported_state(client, "buzzer", "off")
            return

             
     # Show an error if attribute or value are incorrect.
    print("Error: Don't know how to set " + attribute + " to " + value)


# Send the reported state of our actuator tp AWS IoT after it has been triggered, e.g. "led": "on".
def send_reported_state(client, attribute, value):
    # Prepare our sensor data in JSON format.
    payload = {
        "state": {
            "reported": {
                attribute: value,
                "timestamp": datetime.datetime.now().isoformat()
            }
        }
    }
    print("Sending sensor data to AWS IoT...\n" +
          json.dumps(payload, indent=4, separators=(',', ': ')))

    # Publish our sensor data to AWS IoT via the MQTT topic, also known as updating our "Thing Shadow".
    client.publish("$aws/things/" + deviceName + "/shadow/update", json.dumps(payload))
    print("Sent to AWS IoT")




# Print out log messages for tracing.
def on_log(client, userdata, level, buf):
    print("Log: " + buf)


# Start the main program.
main()
