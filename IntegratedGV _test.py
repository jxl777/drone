from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import multiprocessing
import csv
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
from SearchAlgoScript import *
from GVProcess import *
import json
from serial import Serial
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np
from collections.abc import MutableMapping  # Fix for collections error

# Logger Setup
AVC_LOG = 25
logging.addLevelName(AVC_LOG, "AVC")
def log_avc(self, message, *args, **kwargs):
    if self.isEnabledFor(AVC_LOG):
        self._log(AVC_LOG, message, args, **kwargs)
logging.Logger.avc = log_avc
class CustomLevelFilter(logging.Filter):
    def filter(self, record):
        return record.levelno == AVC_LOG
logging.basicConfig(
    filename='flight.log',
    level=AVC_LOG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("FlightLogger")

# Drone & Serial Connection Parameters
espPORT = '/dev/ttyCH341USB1'  # Change based on your system
espBAUDRATE = 115200
ALTITUDE = 4

parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()
SIMULATE_DRONE = not args.livedrone

# Arm and Takeoff Functions
def arm_drone(vehicle):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print("Vehicle armed.")

def move_forward(vehicle, distance):
    print(f"Moving forward {distance} meter(s)")
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(current_location.lat + (distance / 111111), current_location.lon, current_location.alt)
    vehicle.simple_goto(target_location)
    time.sleep(5)  # Adjust for movement timing
    print("Movement complete")

def connect_esp32(port, baudrate):
    try:
        print("Attempting to connect to ESP32...")
        ser = Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Allow time for initialization
        if ser.is_open:
            print("ESP32 connection established successfully.")
            return ser
        else:
            print("Failed to establish ESP32 connection.")
            ser.close()
            return None
    except Exception as e:
        print(f"Error connecting to ESP32: {e}")
        return None

def gv_control(location_queue, isMarkerFound, distance_to_marker_queue):
    vehicle = connectMyCopter()
    print(f"Starting Location: ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
    location_queue.put([vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon])
    print("Heading: ", vehicle.heading)
    logger.avc("Arming Drone")
    arm_drone(vehicle)
    print("Set default airspeed to 3")
    vehicle.airspeed = 3
    logger.avc("UAV START: MOVING FORWARD")
    move_forward(vehicle, 1)  # Move forward 1 meter
    vehicle.close()
    logger.avc("UAV END: MOVEMENT COMPLETE")

def search_algorithm(marker_queue, isMarkerFound):
    while True:
        if not marker_queue.empty():
            marker_id = marker_queue.get()
            with isMarkerFound.get_lock():
                isMarkerFound.value = (marker_id == 0)

def comms(ser, isMarkerFound, location_queue):
    while True:
        if not location_queue.empty():
            locationTuple = location_queue.get()
            data = str(locationTuple) + str(isMarkerFound.value)
            ser.write(data.encode('utf-8'))
            print(f"Sent: {data}")
            logger.avc(f"Sent From Jetson: {data}")
            if isMarkerFound.value:
                logger.avc(f"ArUco Marker Found At {str(locationTuple)}")

if __name__ == "__main__":
    try:
        ser = connect_esp32(espPORT, espBAUDRATE)
        if ser is None:
            exit(1)
        marker_queue = multiprocessing.Queue()
        location_queue = multiprocessing.Queue()
        distance_to_marker_queue = multiprocessing.Queue()
        isMarkerFound = multiprocessing.Value('b', False)
        flight_process = multiprocessing.Process(target=gv_control, args=(location_queue, isMarkerFound, distance_to_marker_queue))
        flight_process.start()
        search_process = multiprocessing.Process(target=search_algorithm, args=(marker_queue, isMarkerFound))
        search_process.start()
        comms_process = multiprocessing.Process(target=comms, args=(ser, isMarkerFound, location_queue))
        comms_process.start()
        search_process.join()
        comms_process.join()
        flight_process.join()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print("Closed serial connection.")
