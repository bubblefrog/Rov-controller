#!/usr/bin/env python

import asyncio
import websockets
import json

# Command received from interface
# {
#     "thrust":0,
#     "yaw":0,
#     "translate":0,
#     "vertical":0,
#     "lights":False,
#     "reportRate":1
# }

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
                rovState["lights"]=data["lights"]
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

async def controlLoop():
    while True:
        await asyncio.sleep(0.1)
        rovState["motors"][0] = commandedMovement["vertical"]
        rovState["motors"][1] = commandedMovement["vertical"]

        rovState["motors"][2] = max(min(commandedMovement["thrust"] + commandedMovement["yaw"],1),-1)
        rovState["motors"][3] = max(min(commandedMovement["thrust"] - commandedMovement["yaw"],1),-1)

async def main():
    controlLoopTask = asyncio.create_task(controlLoop())
    async with websockets.serve(handleConnection, "localhost", 8765):
        reportLoopTask = asyncio.create_task(reportLoop())
        commandLoopTask = asyncio.create_task(commandLoop())
        await controlLoopTask
        await reportLoopTask
        await commandLoopTask

if __name__ == "__main__":
    asyncio.run(main())