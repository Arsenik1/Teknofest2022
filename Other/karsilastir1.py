    print(drone.commands.count)
    drone.commands.next=0 
    nextwaypoint=0 
    print("Going to the water pool...")
    drone.mode = VehicleMode("AUTO")
    drone.commands.next=0 
    nextwaypoint=0
    while True:
        nextwaypoint=drone.commands.next
        if nextwaypoint == 2:
            break
        time.sleep(0.0001)
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