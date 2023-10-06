import threading
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math

# Constants
PERCENT_CENTER_RECT  = 0.20 # For calculating the center rectangle's size
PERCENT_TARGET_RADIUS = 0.2 * PERCENT_CENTER_RECT # Minimum target radius to follow
HOVERING_ALTITUDE    = 7.0 # Altitude in meters to which the drone will perform its tasks
NUM_FILT_POINTS      = 20 # Number of filtering points for the Moving Average Filter
DESIRED_IMAGE_HEIGHT = 480 # A smaller image makes the detection less CPU intensive
SPEED_RATE = 0.02

duration = 10
N_coord = 0
E_coord = 0
vx=0
vy=0
vz=0
redAreaCenter = LocationGlobalRelative(40.2301431,29.0095451,HOVERING_ALTITUDE)
rectflag = False
redAreaTime= False
D_coord = -HOVERING_ALTITUDE # The drone will always detect and track at HOVERING_ALTITUDE
yaw_angle = 0
detected = False
frame=None
# A dictionary of two empty buffers (arrays) for the Moving Average Filter
filt_buffer = {'width':[], 'height':[]}

# A dictionary of general parameters derived from the camera image size,
# which will be populated later with the 'get_image_params' function
params = {'image_height':None, 'image_width': None,'resized_height':None,'resized_width': None,
    'x_ax_pos':None, 'y_ax_pos':None, 'cent_rect_half_width':None, 'cent_rect_half_height': None,
    'cent_rect_p1': None, 'cent_rect_p2': None, 'scaling_factor':None, 'min_tgt_radius':None}


### ---------- This is the application's 'main' asynchronous function ----------
def main(x):
    global N_coord,E_coord,D_coord,yaw_angle,vx,vy,detected,rectflag,redAreaCenter,frame
    """ Detects a target by using color range segmentation and follows it
    by using Offboard control and position NED coordinates. """
    
    # Open the video camera
    if(x == 0):
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
            if (drone.location.global_relative_frame.alt>=HOVERING_ALTITUDE-0.30):
                break
        
        print("Takeoff Success!")
    if x == 1:

        # Check if the camera opened correctly
        if vid_cam.isOpened() is False: 
            print('[ERROR] Couldnt open the camera.')
            return
            
        print('-- Camera opened successfully')

        # Compute general parameters
        get_image_params(vid_cam)

    # Infinite detect-follow loop
    while x == 1:
        if redAreaTime == True:
            redAreaCenter = LocationGlobalRelative(drone.location.global_relative_frame.lat ,drone.location.global_relative_frame.lon,5)
            print("mainden cıktı")
            break
        # Get the target coordinates (if any target was detected)
        tgt_cam_coord, frame, contour = get_target_coordinates(vid_cam)
     
        # If a target was found, filter their coordinates
        if tgt_cam_coord['width'] is not None and tgt_cam_coord['height'] is not None:
            # Apply Moving Average filter to target camera coordinates
            #tgt_filt_cam_coord = moving_average_filter(tgt_cam_coord)
            detected = True
            drone.mode=VehicleMode("GUIDED")

        # No target was found, set target camera coordinates to the Cartesian origin,
        # so the drone doesn't move
        else:
            # The Cartesian origin is where the x and y Cartesian axes are located
            # in the image, in pixel units
            tgt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']} # Needed just for drawing objects
            tgt_filt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']}
            detected = False
            #vx=0
            #vy=0

        # Convert from camera coordinates to Cartesian coordinates (in pixel units)
        tgt_cart_coord = {'x':(tgt_cam_coord['width'] - params['y_ax_pos']),
                          'y':(params['x_ax_pos'] - tgt_cam_coord['height'])}

        # Compute scaling conversion factor from camera coordinates in pixel units
        # to Cartesian coordinates in meters
        COORD_SYS_CONV_FACTOR = 0.1

        # If the target is outside the center rectangle, compute North and East coordinates 
        if abs(tgt_cart_coord['x']) > params['cent_rect_half_width'] or \
        abs(tgt_cart_coord['y']) > params['cent_rect_half_height']:
            # Compute North, East coordinates applying "camera pixel" to Cartesian conversion factor
            E_coord = tgt_cart_coord['x'] * COORD_SYS_CONV_FACTOR
            N_coord = tgt_cart_coord['y'] * COORD_SYS_CONV_FACTOR
            rectflag = False
                # D_coord, yaw_angle don't change
            if N_coord < 0:
                vx = SPEED_RATE * N_coord
            elif N_coord ==0:
                vx=0
            else:
                vx = SPEED_RATE * N_coord
            if E_coord < 0:
                vy = SPEED_RATE * E_coord
            elif E_coord ==0:
                vy=0
            else:
                vy = SPEED_RATE * E_coord
        else:
            vx=0
            vy=0
            if detected == True:
                rectflag = True
        # Command the drone to the current NED + Yaw pose
        #print(N_coord,E_coord,D_coord,yaw_angle)
        #set_position(N_coord,E_coord,D_coord,yaw_angle)
        #threading.Thread(target=set_position).start()
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        # Draw objects over the detection image frame just for visualization
        frame = draw_objects(tgt_cam_coord, tgt_filt_cam_coord, frame, contour)

        # Show the detection image frame on screen
        #cv2.imshow("Detect and Track", frame)
        time.sleep(0.0001)

        # Catch aborting key from computer keyboard
        # If the 'q' key is pressed, break the 'while' infinite loop
            

def goster():
    while True:
        time.sleep(0.0001)
def measureTime():
    global redAreaTime
    while True:
        start = time.time()
        while detected == True and rectflag == True :
            if time.time() - start > 1.80:
                redAreaTime = True
                break
            time.sleep(0.15)
        if redAreaTime == True:
            break
        time.sleep(0.0001)
    
def set_position():
    """
    Move drone in direction based on specified velocity vectors.
    """  
    while redAreaTime == False:
        
        if detected == True or (detected == False):
            #print(E_coord,N_coord,D_coord)
            print(vx,vy,vz)
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
            time.sleep(0.3)
            if redAreaTime == True:       
                break
            # Controls the mavlink message send rate, do not set lower than 0.3
        time.sleep(0.0001)


def addWaypoint(cmds,lat,lon,alt):
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the drone is already in the air.
    #cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point = LocationGlobalRelative(lat, lon,alt)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, point.alt))   
    #print(" Upload new commands to drone")
    cmds.upload()
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
        0, math.radians(0))
    drone.send_mavlink(msg)
    time.sleep(0.5)

if __name__ == "__main__":
    connection = "127.0.0.1:14550"
    #connection = "127.0.0.1:14550"
    drone = connect(connection,wait_ready=True,timeout=100)
    processflag = False
    main(0)
    t1 = threading.Thread(target = set_position)
    t2 = threading.Thread(target = main,args = (1,))
    t3 = threading.Thread(target=measureTime)
    t4 = threading.Thread(target = set_position)
    t5 = threading.Thread(target = main,args = (1,))
    t6 = threading.Thread(target = measureTime)
    waterPool = LocationGlobalRelative(40.2304026,29.009555,2.7)
    waterPoolHigh = LocationGlobalRelative(40.2304026,29.009555,7)
    cmds = drone.commands
    cmds.clear() 
    addWaypoint(cmds,-35.36311705,149.16511499,HOVERING_ALTITUDE)#duz git #dummy
    addWaypoint(cmds,-35.36311705,149.16511499,HOVERING_ALTITUDE)#duz git
    addWaypoint(cmds,-35.36336976,149.16516175,HOVERING_ALTITUDE)#sari çizgi
    #red area is in 3 and 4th waypoints
    addWaypoint(cmds,-35.36336976,149.16532579,HOVERING_ALTITUDE)#kirmizi sonu
    addWaypoint(cmds,-35.36320065,149.16549293,HOVERING_ALTITUDE)#direge capraz
    addWaypoint(cmds,-35.36287757,149.16511533,HOVERING_ALTITUDE)#asagi capraz
    addWaypoint(cmds,-35.36284728,149.16526699,HOVERING_ALTITUDE)#cizgiyi gec
    addWaypoint(cmds,-35.36313250,149.16519580,HOVERING_ALTITUDE) #dummy
    print("Starting first Turn")
    print("Before: ",redAreaCenter.lat,redAreaCenter.lon)
    # Set mode to AUTO to start mission
    drone.mode = VehicleMode("AUTO")
    # Reset mission set to first (0) waypoint
    drone.commands.next=0
    drone.groundspeed = 8
    nextwaypoint = 0
    while True:
        #print("Current Point: ",drone.commands[nextwaypoint-1])
        nextwaypoint=drone.commands.next
        #print(f"COMMAND NEXT {drone.commands.next}")
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        if nextwaypoint==3 and processflag != True :#Skip to next waypoint
            drone.groundspeed = 4
            processflag = True
            t1.start()
            t2.start()
            t3.start()
            t2.join() 
            t1.join()
            drone.mode=VehicleMode("AUTO")
            drone.groundspeed = 8
            drone.commands.next = 3
        if nextwaypoint==6: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break
        time.sleep(0.0001)
    drone.groundspeed = 8
    drone.mode = VehicleMode("GUIDED")
    print("After: ",redAreaCenter.lat,redAreaCenter.lon)
    print("Starting second turn...")
    cmds = drone.commands
    cmds.clear()
    print(drone.commands.count)
    processflag=False
    addWaypoint(cmds,-35.36284728,149.16526699,HOVERING_ALTITUDE)#cizgiyi gec
    addWaypoint(cmds,-35.36331676,149.16523294,HOVERING_ALTITUDE)
    addWaypoint(cmds,waterPool.lat,waterPool.lon,HOVERING_ALTITUDE)
    addWaypoint(cmds,waterPool.lat,waterPool.lon,HOVERING_ALTITUDE) #dummy
    print(drone.commands.count)
    drone.commands.next=0 
    nextwaypoint=0 
    print("Going to the water pool...")
    drone.mode = VehicleMode("AUTO")
    drone.commands.next=0 
    nextwaypoint=0
    while True:
        nextwaypoint=drone.commands.next
        if nextwaypoint == 4:
            print ("Started taking water...aaaaaaaaaaaaaaaaaaaaa")
            break
        time.sleep(0.0001)
    drone.mode = VehicleMode("GUIDED")
    print("Taking some water...")
    time.sleep(2)
    #drone.simple_goto(waterPool,1)
    cmds = drone.commands
    cmds.clear()
    processflag=False
    addWaypoint(cmds,redAreaCenter.lat,redAreaCenter.lon,HOVERING_ALTITUDE)
    addWaypoint(cmds,redAreaCenter.lat,redAreaCenter.lon,HOVERING_ALTITUDE)#dummy
    # Reset mission set to first (0) waypoint
    print("Going to the red area")
    drone.groundspeed=3
    PERCENT_CENTER_RECT = 0.25
    SPEED_RATE = 0.01
    redAreaTime=False
    detected = False
    rectflag=False
    t4.start()
    t5.start()
    t6.start()
    drone.commands.next=0
    nextwaypoint=0
    drone.mode = VehicleMode("AUTO")
    drone.groundspeed=3
    t5.join()
    t4.join()
    cmds = drone.commands
    cmds.clear() 
    processflag=False
    addWaypoint(cmds,40.2297977,29.0091231,HOVERING_ALTITUDE)#kirmizi sonu #dummy
    addWaypoint(cmds,40.2297977,29.0091231,HOVERING_ALTITUDE)#kirmizi sonu
    addWaypoint(cmds,40.2296865,29.0093917,HOVERING_ALTITUDE)#direge capraz
    addWaypoint(cmds,40.2297908,29.0094999,HOVERING_ALTITUDE)#asagi capraz
    addWaypoint(cmds,40.2303817,29.009775,HOVERING_ALTITUDE)#land
    addWaypoint(cmds,40.2303817,29.009775,HOVERING_ALTITUDE) #dummy
    # Reset mission set to first (0) waypoint
    print("Finishing mission...")
    drone.groundspeed=8
    drone.mode = VehicleMode("AUTO")
    drone.commands.next=0
    nextwaypoint=0
    drone.groundspeed=8
    while True:
        nextwaypoint=drone.commands.next
        if nextwaypoint==6:
            break
        time.sleep(0.0001)
    print("Landing...")
    time.sleep(5)
    drone.mode  = VehicleMode("LAND")
    drone.close()