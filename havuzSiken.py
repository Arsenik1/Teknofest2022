import threading
import cv2
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
#import RPi.GPIO as GPIO

# Constants
PERCENT_CENTER_RECT  = 0.20 # For calculating the center rectangle's size
PERCENT_TARGET_RADIUS = 0.2 * PERCENT_CENTER_RECT # Minimum target radius to follow
HOVERING_ALTITUDE    = 9.0 # Altitude in meters to which the drone will perform its tasks
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

class colorFlag:
    y=0
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

        vid_cam = cv2.VideoCapture(0)
        #fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

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
            if colorFlag.y == 0:
                redAreaCenter = LocationGlobalRelative(drone.location.global_relative_frame.lat ,drone.location.global_relative_frame.lon,5)
            cv2.destroyAllWindows()
            #out.release()
            vid_cam.release()
            print("mainden cıktı")
            break
        # Get the target coordinates (if any target was detected)
        tgt_cam_coord, frame, contour = get_target_coordinates(vid_cam,colorFlag.y)
     
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
        cv2.imshow("Detect and Track", frame)
        #out.write(frame)
        time.sleep(0.0001)

        # Catch aborting key from computer keyboard
        key = cv2.waitKey(1) & 0xFF
        # If the 'q' key is pressed, break the 'while' infinite loop
        if key == ord("q"):
            
            break

def goster():
    while True:
        cv2.imshow("Detect and Track", frame)
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
def suyuBirak():
    print("suyu birakiyor....")
    servoPIN = 13
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPIN, GPIO.OUT)
    p = GPIO.PWM(servoPIN, 50) # GPIO 13 for PWM with 50Hz
    p.start(2.5) # Initialization
    p.ChangeDutyCycle(5)
    time.sleep(8.0)
    p.stop()
    GPIO.cleanup()
def set_position():
    """
    Move drone in direction based on specified velocity vectors.
    """  
    while redAreaTime == False:
        
        if detected == True or (detected == False):
            #print(E_coord,N_coord,D_coord)
            print("x:",vx,"y:",vy,"z:",vz)
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


def get_target_coordinates(vid_cam,redOrBlue):
    """ Detects a target by using color range segmentation and returns its 'camera pixel' coordinates."""

    # Use the 'threshold_inRange.py' script included with the code to get
    # your own bounds with any color
    # To detect a blue target:
    #real
    #HSV_LOWER_BOUND = (134, 95, 171)
    #HSV_UPPER_BOUND = (179, 193, 255)
    #simulation
    HSV_LOWER_BOUND = (160,50,50)
    HSV_UPPER_BOUND = (179, 255, 255)
    if redOrBlue == 1:
        HSV_LOWER_BOUND = (100,150,50)
        HSV_UPPER_BOUND = (140, 255, 255)       
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

    """
    # Draw filtered centroid as a blue dot, including coordinate values
    cv2.circle(frame, (filt_cam_coord['width'], filt_cam_coord['height']), 5, (255, 30, 30), -1)
    cv2.putText(frame, str(x_filt_cart_coord) + ", " + str(y_filt_cart_coord), 
        (filt_cam_coord['width'] + 25, filt_cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 30, 30), 1)
    """
    return frame # Return the image frame with all drawn objects

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
    #connection = "/dev/ttyACM1"
    connection = "127.0.0.1:14550"
    drone = connect(connection,wait_ready=False,timeout=100)
    processflag = False
    main(0)
    colorFlag.y = 0
    t1 = threading.Thread(target = set_position)
    t2 = threading.Thread(target = main,args = (1,))
    t3 = threading.Thread(target=measureTime)
    t4 = threading.Thread(target = set_position)
    t5 = threading.Thread(target = main,args = (1,))
    t6 = threading.Thread(target = measureTime)
    t7 = threading.Thread(target = set_position)
    t8 = threading.Thread(target = main,args = (1,))
    t9 = threading.Thread(target = measureTime)
    waterPool = LocationGlobalRelative(-35.3630619,149.1652179,6)
    cmds = drone.commands
    #Cleared all commands.
    #Commands will come from mission planner.

    print("Starting first Turn")
    print("Before: ",redAreaCenter.lat,redAreaCenter.lon)
    drone.mode = VehicleMode("AUTO")
    # Set mode to AUTO to start mission
    # Reset mission set to first (0) waypoint
    drone.commands.next=0
    drone.groundspeed = 3.00
    nextwaypoint = 0
    while True:
        #print("Current Point: ",drone.commands[nextwaypoint-1])
        nextwaypoint=drone.commands.next
        #print(f"COMMAND NEXT {drone.commands.next}")
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        if nextwaypoint==4 and processflag != True:
            drone.groundspeed = 2.00
        if nextwaypoint==5 and processflag != True :#Skip to next waypoint
            drone.groundspeed = 1.00
            processflag = True
            t1.start()
            t2.start()
            t3.start()
            t2.join()
            t1.join()
            drone.mode=VehicleMode("AUTO")
            drone.groundspeed = 3.00
            print("Red found")
            drone.commands.next = 5
        if nextwaypoint==8: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break
        time.sleep(0.0001)
    drone.groundspeed = 4.00
    drone.mode = VehicleMode("GUIDED")
    print("After: ",redAreaCenter.lat,redAreaCenter.lon)
    print("Starting second turn...")
    cmds = drone.commands
    cmds.clear()
    print(drone.commands.count)
    processflag=False
    addWaypoint(cmds,waterPool.lat,waterPool.lon,HOVERING_ALTITUDE) #dummy
    addWaypoint(cmds,waterPool.lat,waterPool.lon,HOVERING_ALTITUDE)
    addWaypoint(cmds,waterPool.lat,waterPool.lon,HOVERING_ALTITUDE) #dummy
    print(drone.commands.count)
    drone.commands.next=0 
    nextwaypoint=0 
    print("Going to the water pool...")
    colorFlag.y = 1
    drone.mode = VehicleMode("AUTO")
    drone.commands.next=0 
    nextwaypoint=0
    while True:
        nextwaypoint=drone.commands.next
        if nextwaypoint == 2:
            break
        time.sleep(0.0001)
    processflag = True
    redAreaTime=False
    detected = False
    rectflag=False
    t7.start()
    t8.start()
    t9.start()
    t8.join()
    t7.join()
    processflag = False
    drone.mode = VehicleMode("GUIDED")
    print("Taking some water...")
    time.sleep(6)
    #drone.simple_goto(waterPool,1)
    while True:
        take_water(0,0,0.4) #0.2 before
        print(drone.location.global_relative_frame.alt)
        if drone.location.global_relative_frame.alt <= 2.3:
            break
    take_water(0,0,0)
    print("Taking water is done. Rising...")
    colorFlag.y = 0
    print("Running motors to avoid water lose.")
    print("Running motors is done.")
    HOVERING_ALTITUDE = 6
    time.sleep(5)
    while True:
        take_water(0,0,-0.4) #0.2 before
        print(drone.location.global_relative_frame.alt)
        if drone.location.global_relative_frame.alt >= HOVERING_ALTITUDE:
            break
    cmds = drone.commands
    cmds.clear()
    processflag=False
    addWaypoint(cmds,40.8118305,29.3604764,HOVERING_ALTITUDE)#dummy
    addWaypoint(cmds,40.8118305,29.3604764,HOVERING_ALTITUDE)#1
    addWaypoint(cmds,40.8118731,29.3605569,HOVERING_ALTITUDE)#2
    addWaypoint(cmds,redAreaCenter.lat,redAreaCenter.lon,HOVERING_ALTITUDE)#kirmizi
    addWaypoint(cmds,redAreaCenter.lat,redAreaCenter.lon,HOVERING_ALTITUDE)#dummy
    # Reset mission set to first (0) waypoint
    print("Going to the red area")
    drone.groundspeed=1
    PERCENT_CENTER_RECT = 0.25
    SPEED_RATE = 0.01
    redAreaTime=False
    detected = False
    rectflag=False
    drone.commands.next=0
    nextwaypoint=0
    drone.mode = VehicleMode("AUTO")
    drone.groundspeed=1
    while True:
        #print("Current Point: ",drone.commands[nextwaypoint-1])
        nextwaypoint=drone.commands.next
        #print(f"COMMAND NEXT {drone.commands.next}")
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        if nextwaypoint==4 and processflag != True :#Skip to next waypoint
            drone.groundspeed = 1
            processflag = True
            t4.start()
            t5.start()
            t6.start()
            t5.join()
            t4.join()
            drone.mode=VehicleMode("AUTO")
            drone.groundspeed = 4.00
            print("Red found")
            drone.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit. #NEXTWAYPOİNTTE SIKINTI CIKABILIR DENEDIKTEN SONRA DUZELT
            print("ikinci kirmizi bulundu!!")
            break
        time.sleep(0.0001)
    print("Water is pouring...")
    suyuBirak()
    time.sleep(5)
    cmds = drone.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()
    processflag=False
    print("su birakildi devamke...")
    time.sleep(1)
    addWaypoint(cmds,40.8120665, 29.3605703,HOVERING_ALTITUDE)#kirmizi sonu #dummy
    addWaypoint(cmds,40.8120665, 29.3605703,HOVERING_ALTITUDE)#kirmizi sonu
    addWaypoint(cmds,40.8120918, 29.3604315,HOVERING_ALTITUDE)#direge capraz
    addWaypoint(cmds,40.812002, 29.3602967,HOVERING_ALTITUDE)#asagi capraz
    addWaypoint(cmds,40.812002, 29.3602967,HOVERING_ALTITUDE)#land
    addWaypoint(cmds,40.812002, 29.3602967,HOVERING_ALTITUDE)#land #dummy
    # Reset mission set to first (0) waypoint
    print("Finishing mission...")
    cmds.upload()
    drone.groundspeed=2
    drone.mode = VehicleMode("AUTO")
    time.sleep(2)
    drone.mode = VehicleMode("GUIDED")
    time.sleep(2)
    drone.mode = VehicleMode("AUTO")
    print("donmeye girmek uzere")
    drone.commands.next=0
    nextwaypoint=0
    drone.groundspeed=4
    while True:
        nextwaypoint=drone.commands.next
        print("donuyor")
        if nextwaypoint==6:
            break
        time.sleep(0.0001)
    print("Landing...")
    time.sleep(5)
    drone.mode  = VehicleMode("LAND")
    drone.close()
