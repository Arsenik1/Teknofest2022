import threading
import cv2
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import math
import time

# Constants
PERCENT_CENTER_RECT  = 0.20 # For calculating the center rectangle's size
PERCENT_TARGET_RADIUS = 0.5 * PERCENT_CENTER_RECT # Minimum target radius to follow
HOVERING_ALTITUDE    = 5.0 # Altitude in meters to which the drone will perform its tasks
NUM_FILT_POINTS      = 20 # Number of filtering points for the Moving Average Filter
DESIRED_IMAGE_HEIGHT = 480 # A smaller image makes the detection less CPU intensive
duration = 10
N_coord = 0
E_coord = 0
vx=0
vy=0
vz=0
redAreaCenter = LocationGlobalRelative( 40.8117235,29.3601398,5)
rectflag = False
redAreaTime= False
D_coord = -HOVERING_ALTITUDE # The drone will always detect and track at HOVERING_ALTITUDE
yaw_angle = 0
detected = False
# A dictionary of two empty buffers (arrays) for the Moving Average Filter
filt_buffer = {'width':[], 'height':[]}

# A dictionary of general parameters derived from the camera image size,
# which will be populated later with the 'get_image_params' function
params = {'image_height':None, 'image_width': None,'resized_height':None,'resized_width': None,
    'x_ax_pos':None, 'y_ax_pos':None, 'cent_rect_half_width':None, 'cent_rect_half_height': None,
    'cent_rect_p1': None, 'cent_rect_p2': None, 'scaling_factor':None, 'min_tgt_radius':None}


### ---------- This is the application's 'main' asynchronous function ----------
def main(x):
    global N_coord,E_coord,D_coord,yaw_angle,temp,vx,vy,detected,rectflag,redAreaCenter
    """ Detects a target by using color range segmentation and follows it
    by using Offboard control and position NED coordinates. """
    
    # Open the video camera
    if(x == 0):

    
        # print(f"-- Original image width, height: {params['image_width']}, {params['image_height']}")

        # Get a reference to a 'System' object, which represents the drone,
        # and open a connection to it. The system address used here is the default
        # address for a simulated drone running in the same machine machine where 
        # this code will run (localhost)
        # To run with SITL simulation
        # ------ To run the code with a real drone connected to the PC via telemetry modules ------ #
        # drone.connect(system_address="serial:///dev/ttyUSB0:57600") 
        # CAUTION: Color range segmentation is not the best approach to detect and track objects
        #          This example is just a didactic proof of concept. Don't run it with a real drone
        #          unless you have good experience flying real drones and know what you are doing.
        # ----------------------------------------------------------------------------------------- #
        


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
    if x == 1:

        vid_cam = cv2.VideoCapture(0)
        
        # Let the camera warm up
        # asyncio.sleep(2)

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
            cv2.destroyAllWindows()
            print("Goruntu islemeden cikti")
            break
        # Get the target coordinates (if any target was detected)
        tgt_cam_coord, frame, contour = get_target_coordinates(vid_cam)
     
        # If a target was found, filter their coordinates
        if tgt_cam_coord['width'] is not None and tgt_cam_coord['height'] is not None:
            # Apply Moving Average filter to target camera coordinates
            tgt_filt_cam_coord = moving_average_filter(tgt_cam_coord)
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
            vx=0
            vy=0

        # Convert from camera coordinates to Cartesian coordinates (in pixel units)
        tgt_cart_coord = {'x':(tgt_filt_cam_coord['width'] - params['y_ax_pos']),
                          'y':(params['x_ax_pos'] - tgt_filt_cam_coord['height'])}

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
                vx = -0.35
            elif N_coord ==0:
                vx=0
            else:
                vx = 0.35
            if E_coord < 0:
                vy = -0.35
            elif E_coord ==0:
                vy=0
            else:
                vy = 0.35
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
        cv2.imshow("Detect and Track", frame)

        # Catch aborting key from computer keyboard
        key = cv2.waitKey(1) & 0xFF
        # If the 'q' key is pressed, break the 'while' infinite loop
        if key == ord("q"):
            
            break

def measureTime():
    global redAreaTime
    while True:
        start = time.time()
        while detected == True and rectflag == True :
            if time.time() - start > 2:
                redAreaTime = True
                break
            time.sleep(0.25)
        if redAreaTime == True:
            break
        time.sleep(0.0001)
    
def set_position():
    """
    Move drone in direction based on specified velocity vectors.
    """  
    while True:

        if detected == True :
            #print(N_coord,E_coord,D_coord)
            #print(vy,vx,vz)
            msg = drone.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
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
                print("Set position bitti.")
                break
            # Controls the mavlink message send rate, do not set lower than 0.3
                
            #drone.simple_goto(LocationGlobalRelative(0,0))
        else:
            time.sleep(0.00001)


def get_image_params(vid_cam):
    """ Computes useful general parameters derived from the camera image size."""

    # Grab a frame and get its size
    is_grabbed, frame = vid_cam.read()
    params['image_height'], params['image_width'], _ = frame.shape

    # Compute the scaling factor to scale the image to a desired size
    if params['image_height'] != DESIRED_IMAGE_HEIGHT:
        params['scaling_factor'] = round((DESIRED_IMAGE_HEIGHT / params['image_height']), 2) # Rounded scaling factor
        
    else:
        params['scaling_factor'] = 1

    print("params['scaling_factor']: ", params['scaling_factor'])

    # Compute resized width and height and resize the image
    params['resized_width'] = int(params['image_width'] * params['scaling_factor'])
    params['resized_height'] = int(params['image_height'] * params['scaling_factor'])
    dimension = (params['resized_width'], params['resized_height'])
    frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

    # Compute the center rectangle's half width and half height
    params['cent_rect_half_width'] = round(params['resized_width'] * (0.5 * PERCENT_CENTER_RECT)) # Use half percent (0.5)
    params['cent_rect_half_height'] = round(params['resized_height'] * (0.5 * PERCENT_CENTER_RECT)) # Use half percent (0.5)

    # Compute the minimum target radius to follow. Smaller detected targets will be ignored
    params['min_tgt_radius'] = round(params['resized_width'] * PERCENT_TARGET_RADIUS)

    # Compute the position for the X and Y Cartesian coordinates in camera pixel units
    params['x_ax_pos'] = int(params['resized_height']/2 - 1)
    params['y_ax_pos'] = int(params['resized_width']/2 - 1)

    # Compute two points: p1 in the upper left and p2 in the lower right that will be used to
    # draw the center rectangle iin the image frame
    params['cent_rect_p1'] = (params['y_ax_pos'] - params['cent_rect_half_width'], 
                              params['x_ax_pos'] - params['cent_rect_half_height'])
    params['cent_rect_p2'] = (params['y_ax_pos'] + params['cent_rect_half_width'], 
                              params['x_ax_pos'] + params['cent_rect_half_height'])

    return


def get_target_coordinates(vid_cam):
    """ Detects a target by using color range segmentation and returns its 'camera pixel' coordinates."""

    # Use the 'threshold_inRange.py' script included with the code to get
    # your own bounds with any color
    # To detect a blue target:
    
    #HSV_LOWER_BOUND = (134, 95, 171)
    #HSV_UPPER_BOUND = (179, 193, 255)
    HSV_LOWER_BOUND = (161, 155, 84)
    HSV_UPPER_BOUND = (179, 255, 255)
    # Grab a frame in BGR (Blue, Green, Red) space color
    is_grabbed, frame = vid_cam.read()

    # Resize the image frame for the detection process, if needed
    if params['scaling_factor'] != 1:
        dimension = (params['resized_width'], params['resized_height'])
        frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

    # Blur the image to remove high frequency content
    blurred = cv2.GaussianBlur(frame, (11, 11), 0) 
    
    # Change color space from BGR to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Histogram equalisation to minimize the effect of variable lighting
    # hsv[:, :, 0] = cv2.equalizeHist(hsv[:, :, 0]) # on the H-channel
    # hsv[:, :, 1] = cv2.equalizeHist(hsv[:, :, 1]) # on the S-channel
    # hsv[:, :, 2] = cv2.equalizeHist(hsv[:, :, 2]) # on the V-channel

    # Get a mask with all the pixels inside our defined color boundaries
    mask = cv2.inRange(hsv, HSV_LOWER_BOUND, HSV_UPPER_BOUND)

    # Erode and dilate to remove small blobs
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)

    # Find all contours in the masked image
    contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Centroid coordinates to be returned:
    cX = None
    cY = None

    # To save the larges contour, presumably the detected object
    largest_contour = None

    # Check if at least one contour was found
    if len(contours) > 0:
        # Get the largest contour of all posibly detected
        largest_contour = max(contours, key=cv2.contourArea)

        # Compute the radius of an enclosing circle aorund the largest contour
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # Compute centroid only if contour radius is larger than 0.5 half the center rectangle
        if radius > params['min_tgt_radius']:
            # Compute contour raw moments
            M = cv2.moments(largest_contour)
            # Get the contour's centroid
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

    # Return centroid coordinates (camera pixel units), the analized frame and the largest contour
    return {'width':cX, 'height':cY}, frame, largest_contour


def moving_average_filter(coord):
    """ Applies Low-Pass Moving Average Filter to a pair of coordinates."""

    # Append new coordinates to filter buffers
    filt_buffer['width'].append(coord['width'])
    filt_buffer['height'].append(coord['height'])

    # If the filters were full already with a number of NUM_FILT_POINTS values, 
    # discard the oldest value (FIFO buffer)
    if len(filt_buffer['width']) > NUM_FILT_POINTS:
        filt_buffer['width'] = filt_buffer['width'][1:]
        filt_buffer['height'] = filt_buffer['height'][1:]
    
    # Compute filtered camera coordinates
    N = len(filt_buffer['width']) # Get the number of values in buffers (will be < NUM_FILT_POINTS at the start)

    # Sum all values for each coordinate
    w_sum = sum( filt_buffer['width'] )
    h_sum = sum( filt_buffer['height'] )
    # Compute the average
    w_filt = int(round(w_sum / N))
    h_filt = int(round(h_sum / N))

    # Return filtered coordinates as a dictionary
    return {'width':w_filt, 'height':h_filt}


def draw_objects(cam_coord, filt_cam_coord, frame, contour):
    """ Draws visualization objects from the detection process.
    Position coordinates of every object are always in 'camera pixel' units"""

    # Draw the Cartesian axes
    cv2.line(frame, (0, params['x_ax_pos']), (params['resized_width'], params['x_ax_pos']), (0, 128, 255), 1)
    cv2.line(frame, (params['y_ax_pos'], 0), (params['y_ax_pos'], params['resized_height']), (0, 128, 255), 1)
    cv2.circle(frame, (params['y_ax_pos'], params['x_ax_pos']), 1, (255, 255, 255), -1)
    
    # Draw the center (tolerance) rectangle
    cv2.rectangle(frame, params['cent_rect_p1'], params['cent_rect_p2'], (0, 178, 255), 1)

    # Draw the detected object's contour, if any
    #cv2.drawContours(frame, [contour], 0, (0, 0, 255), 5)

    # Compute Cartesian coordinates of unfiltered detected object's centroid
    x_cart_coord = cam_coord['width'] - params['y_ax_pos']
    y_cart_coord = params['x_ax_pos'] - cam_coord['height']

    # Compute Cartesian coordinates of filtered detected object's centroid
    x_filt_cart_coord = filt_cam_coord['width'] - params['y_ax_pos']
    y_filt_cart_coord = params['x_ax_pos'] - filt_cam_coord['height']

    # Draw unfiltered centroid as a red dot, including coordinate values
    cv2.circle(frame, (cam_coord['width'], cam_coord['height']), 5, (0, 0, 255), -1) 
    cv2.putText(frame, str(x_cart_coord) + ", " + str(y_cart_coord), 
        (cam_coord['width'] + 25, cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Draw filtered centroid as a blue dot, including coordinate values
    cv2.circle(frame, (filt_cam_coord['width'], filt_cam_coord['height']), 5, (255, 30, 30), -1)
    cv2.putText(frame, str(x_filt_cart_coord) + ", " + str(y_filt_cart_coord), 
        (filt_cam_coord['width'] + 25, filt_cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 30, 30), 1)

    return frame # Return the image frame with all drawn objects


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = drone.commands.next
    if nextwaypoint==0:
        return None
    missionitem=drone.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(drone.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the drone.
    """
    cmds = drone.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def addWaypoint(cmds,lat,lon,alt):
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the drone is already in the air.
    #cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point = LocationGlobalRelative(lat, lon,alt)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, 5))   
    print(" Upload new commands to drone")
    cmds.upload()

def takeWater():
    """
    Move drone in direction based on specified velocity vectors.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        3, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, math.radians(0))    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to drone on 1 Hz cycle
    #for x in range(0,duration):
        
    drone.send_mavlink(msg)
    time.sleep(0.5) # Controls the mavlink message send rate, do not set lower than 0.3
        
    #drone.simple_goto(LocationGlobalRelative(0,0))
if __name__ == "__main__":
    #connection = "/dev/ttyACM0"
    connection = "127.0.0.1:14550"
    drone = connect(connection,wait_ready=True,timeout=100)
    drone.airspeed = 2
    drone.groundspeed = 2
    processflag = False
    main(0)
    t1 = threading.Thread(target = set_position)
    t2 = threading.Thread(target = main,args = (1,))
    t3 = threading.Thread(target=measureTime)
    t1.start()
    t2.start()
    t3.start()
    waterPool = LocationGlobalRelative(40.8116662,29.3604854,5)
    print('Create a new mission (for current location)')
    cmds = drone.commands
    print(" Clear any existing commands")
    cmds.clear() 
    addWaypoint(cmds,-35.36342141,149.16526092,5)
    addWaypoint(cmds,-35.36348842,149.16526964,5)
    #red area is in 3 and 4th waypoints
    addWaypoint(cmds,-35.36327019,149.16523781,5)
    addWaypoint(cmds,-35.36313948,149.16522626,5)
    addWaypoint(cmds,-35.36303351,149.16519305,5)
    addWaypoint(cmds,-35.36291811,149.16517139,5)
    addWaypoint(cmds,-35.36290045,149.16516995,5) #dummy
    print("Starting first Turn")
    print("Before: ",redAreaCenter.lat,redAreaCenter.lon)
    # Reset mission set to first (0) waypoint
    drone.commands.next=0
    # Set mode to AUTO to start mission
    drone.mode = VehicleMode("AUTO")
    nextwaypoint = 0
    while True:
        #print("Current Point: ",drone.commands[nextwaypoint-1])
        nextwaypoint=drone.commands.next
        #print(f"COMMAND NEXT {drone.commands.next}")
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        if nextwaypoint==4 and processflag != True :#Skip to next waypoint
            drone.groundspeed=2
            drone.airspeed = 2
            processflag = True
            t2.join() 
            t1.join()  
            drone.airspeed=2
            drone.groundspeed=2
            drone.mode=VehicleMode("AUTO")
            drone.commands.next = 4
        if nextwaypoint==7: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break
    drone.mode = VehicleMode("GUIDED")
    print("After: ",redAreaCenter.lat,redAreaCenter.lon)
    print('Create a new mission (for current location)')
    cmds = drone.commands
    print(" Clear any existing commands")
    cmds.clear() 
    processflag=False
    addWaypoint(cmds,-35.36342141,149.16526092,5)
    addWaypoint(cmds,-35.36348842,149.16526964,5)
    addWaypoint(cmds,waterPool.lat,waterPool.lon,waterPool.alt)
    addWaypoint(cmds,redAreaCenter.lat,redAreaCenter.lon,redAreaCenter.alt)  
    addWaypoint(cmds,-35.36303351,149.16519305,5)
    addWaypoint(cmds,-35.36291811,149.16517139,5)
    addWaypoint(cmds,-35.36290045,149.16516995,5) #dummy
    # Reset mission set to first (0) waypoint
    drone.commands.next=0
    nextwaypoint=0
    drone.mode = VehicleMode("AUTO")
    while True:
        #print("Current Point: ",drone.commands[nextwaypoint-1])
        nextwaypoint=drone.commands.next
        #print(f"COMMAND NEXT {drone.commands.next}")
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        if nextwaypoint==4 and processflag != True:#Skip to next waypoint
            processflag = True
            drone.mode  = VehicleMode("GUIDED")
            tempPool = waterPool
            tempPool.alt = 2
            drone.simple_goto(tempPool)
            #takeWater()
            time.sleep(10)
            drone.mode=VehicleMode("AUTO")
            drone.commands.next = 4
        if nextwaypoint == 5:
            time.sleep(10)
        if nextwaypoint==8: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break

    drone.mode  = VehicleMode("LAND")
    drone.close()
   

    
    
    
    
    
    
    
