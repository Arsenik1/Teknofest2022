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
    time.sleep(1)
    cmds = drone.commands
    cmds.clear()
    processflag=False
    print("su birakildi devamke...")
    time.sleep(1)