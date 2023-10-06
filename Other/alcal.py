from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import RPi.GPIO as GPIO
HOVERING_ALTITUDE    = 7.0
redAreaCenter = LocationGlobalRelative( 40.8118546,29.3602402,HOVERING_ALTITUDE)
waterPool = LocationGlobalRelative(40.8118306,29.3604478,2.8)
waterPoolHigh = LocationGlobalRelative(40.8118306,29.3604478,7)

def takeoff():
    while drone.is_armable==False:
        print("Drone cannot be armed")
        time.sleep(1)

    drone.mode=VehicleMode("GUIDED")

    while drone.mode=="GUIDED":
        print("Switching to GUIDED mode")
        time.sleep(1.5)
    drone.armed=True

    while drone.armed==False:
        print("Waiting for arm...")
        time.sleep(1)

    print("Drone has armed !")
    drone.simple_takeoff(HOVERING_ALTITUDE)
    while True:
        #print(drone.location.global_relative_frame.alt)
        if (drone.location.global_relative_frame.alt>=HOVERING_ALTITUDE):
            break
    
    print("Takeoff Success!")


def take_water(vx,vy,vz):
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        vx, # X velocity in NED frame in m/s
        vy, # Y velocity in NED frame in m/s
        vz, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, math.radians(0))    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to drone on 1 Hz cycle
    #for x in range(0,duration):
        
    drone.send_mavlink(msg)
    time.sleep(3)


if __name__ == "__main__":
    connection = "/dev/ttyACM0"
    #connection = "127.0.0.1:14550"
    drone = connect(connection,wait_ready=True,timeout=100)
    takeoff()
    time.sleep(2)
    print("Taking some water...")
    #drone.simple_goto(waterPoolHigh,1)
    #time.sleep(5)
    #drone.simple_goto(waterPool,1)
    for i in range(0,3):
        set_position(0,0,0.5)
        #time.sleep(1)
    #time.sleep(5)
    set_position(0,0,0.2)
    set_position(0,0,0)
    #drone.mode  = VehicleMode("LAND")
    drone.close()
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)
    GPIO.output(21, True)
    time.sleep(18)
    GPIO.output(21, False)
    drone.simple_goto(waterPoolHigh,1)
    GPIO.output(21, True)
    time.sleep(10)
    GPIO.output(21, False)

    print("Water is pouring...")
    #takeWater()
    drone.simple_goto(redAreaCenter,1)
    time.sleep(8)
    GPIO.output(20, True)
    time.sleep(15)
    GPIO.output(20, False)
    GPIO.cleanup()
    """