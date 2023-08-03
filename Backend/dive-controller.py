#!/usr/bin/env python

import asyncio
import websockets
import json
import math

import time
import board
import digitalio
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit
i2c = busio.I2C(board.SCL, board.SDA)

from mpu6050 import mpu6050

# Command received from interface
# {
#     "thrust":0,
#     "yaw":0,
#     "translate":0,
#     "vertical":0,
#     "lights":False,
#     "reportRate":1
# }

LightsGpio = digitalio.DigitalInOut(board.D17)
LightsGpio.direction = digitalio.Direction.OUTPUT

# LightsGpio.value = True
time.sleep(0.5)
LightsGpio.value = False

Mpu = mpu6050(0x68)
ADC = ADS.ADS1115(i2c)
Servo = ServoKit(channels=16)

Servo.servo[0].set_pulse_width_range(1180, 2180)
Servo.servo[1].set_pulse_width_range(1180, 2180)
Servo.servo[2].set_pulse_width_range(1180, 2180)
Servo.servo[3].set_pulse_width_range(1180, 2180)

commandedMovement ={
    "thrust":0,
    "yaw":0,
    "translate":0,
    "vertical":0,
}

rovState = {
    "lights":False,
    "pitch":0,
    "roll":0,
    "heading":0,
    "depth": 0.0,
    "batteryVoltage":0,
    "ampDraw":0,
    "motors":[0,0,0,0],
    "internalTemp":0,
    "reportRate":1
}

messageTemplate = {
    "status":0,
    "data":""
}

CONNECTIONS = set()

async def handleConnection(websocket):
    CONNECTIONS.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        CONNECTIONS.remove(websocket)

async def reportLoop():
    while True:
        await asyncio.sleep(1/rovState["reportRate"])
        websockets.broadcast(CONNECTIONS, json.dumps(rovState))

async def commandLoop():
    while True:
        await asyncio.sleep(0.1)
        for ws in CONNECTIONS:
            async for message in ws:
                processCommand(message)

def processCommand(command):
    try:
        print (command)
        message = json.loads(command)
        print (message)
        if message["status"]==200:
            #OK was recieved
            data = message["data"]
            print(data)
            if "reportRate" in data:
                rovState["reportRate"]=data["reportRate"]
            if "lights" in data:
                rovState["lights"]=True if data["lights"]==1 else False
            if "thrust" in data:
                commandedMovement["thrust"]=data["thrust"]
            if "yaw" in data:
                commandedMovement["yaw"]=data["yaw"]
            if "translate" in data:
                commandedMovement["translate"]=data["translate"]
            if "vertical" in data:
                commandedMovement["vertical"]=data["vertical"]
        else:
            print("Invalid message received!")
    except:
        print("Something went wrong")
def motorToServoAngle(motor):
    motorBounded = max(min(motor,1),-1)
    return (180/2)*motorBounded+90
async def controlLoop():
    while True:
        await asyncio.sleep(0.001)
        rovState["motors"][0] = commandedMovement["vertical"]
        rovState["motors"][1] = commandedMovement["vertical"]

        rovState["motors"][2] = max(min(commandedMovement["thrust"] + commandedMovement["yaw"],1),-1)
        rovState["motors"][3] = max(min(commandedMovement["thrust"] - commandedMovement["yaw"],1),-1)

        #Interact with hardware
        LightsGpio.value = rovState["lights"]
        try:
            rovState["internalTemp"] = Mpu.getTemp()
            accelerometer_data = Mpu.get_accel_data()
            rovState["roll"] = math.atan2(accelerometer_data['x'], accelerometer_data['z'])
            rovState["pitch"] = math.atan2(-accelerometer_data['y'], math.sqrt(accelerometer_data['x']*accelerometer_data['x'] + accelerometer_data['z']*accelerometer_data['z']))
        except:
            print ("Could not read MPU")
        currentData = AnalogIn(ADC, ADS.P0)
        vbatData = AnalogIn(ADC, ADS.P1)
        depthData = AnalogIn(ADC, ADS.P2)

        rovState["ampDraw"] = (currentData.voltage*1000)/21
        rovState["batteryVoltage"] = vbatData.voltage/0.196428571
        rovState["depth"] = (depthData.voltage/3.3)*101.974-10

        for (val,idx) in zip(rovState["motors"],range(len(rovState["motors"]))):
            Servo.servo[idx].angle = motorToServoAngle(val)
            print ("Motor: " + str(idx) + " set to: " + str(motorToServoAngle(val)))


async def main():
    # Servo.servo[0].angle = 180
    # Servo.servo[1].angle = 180
    # Servo.servo[2].angle = 180
    # Servo.servo[3].angle = 180

    # await asyncio.sleep(4)

    # Servo.servo[0].angle = 0
    # Servo.servo[1].angle = 0
    # Servo.servo[2].angle = 0
    # Servo.servo[3].angle = 0
    # await asyncio.sleep(4)

    # Servo.servo[0].angle = 90
    # Servo.servo[1].angle = 90
    # Servo.servo[2].angle = 90
    # Servo.servo[3].angle = 90
    controlLoopTask = asyncio.create_task(controlLoop())
    async with websockets.serve(handleConnection, None, 8765):
        reportLoopTask = asyncio.create_task(reportLoop())
        commandLoopTask = asyncio.create_task(commandLoop())
        await controlLoopTask
        await reportLoopTask
        await commandLoopTask

if __name__ == "__main__":
    asyncio.run(main())