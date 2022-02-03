#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---- Dronekit Imports ---------------
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

#----- Radar Imports  ----------------

from signal import signal, SIGINT
from srp.utilities import Sleep

from srp.common.transport import SRP_TCPClient
from srp.client import SRP_ControlClientInterface, SRP_DataClientInterface
from srp.packet import *
from srp.packet_id import PacketID
from srp.payload_types import PAYLOAD_TYPES


######################### VEHICLE CONFIG ##################################

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = 'udpin:localhost:14551'


######################### RADAR CONFIG ##################################

hostAddr = '127.0.0.1'

srpControlClient = SRP_ControlClientInterface( hostAddr, 5002 )
srpDataClient = SRP_DataClientInterface( SRP_TCPClient(hostAddr, 29172) )

class App:
    def __init__(self):
        self.running = True
app = App()


"""/** Signal handler */"""
def signalHandler_SIGINT(signal_received, frame):
    if app.running == True:
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        app.running = False
    else:
        print('2nd SIGINT or CTRL-C detected. Exiting now')
        exit(0)


def onStateChange(params):
    print('onStateChange:')
    laststate = params[0]
    newstate = params[1]
    print('\tState changing from',laststate,'to',newstate)

    # if entering running state then start stream client
    if newstate == 'RUNNING':
        srpDataClient.start()
    else:
        srpDataClient.stop()


def onReceivePacket(packet):
    print('onReceivePacket:')
    if packet.getPayloadType()==PacketID.PayloadType.RadarPoint:
        print('\tgot Points')

        #print packet to console
        #packet.print()

      #-------------------------------------------- 
        """
        This part is parsing the radar readings into different variables. 
        Values are from the first row of the packet items, need to update with getting avarage of the values in all packet. 
        """ 
        serial_packet = packet.serialise()
        temp = list(serial_packet.items())

        payload_Items = temp[1] # Getting first raw in payload only...
        measurements = list(payload_Items[1][1].values()) 

        range_meas = measurements[0]
        doppler_meas = measurements[1]
        azimuthAng_meas = measurements[2]
        elevationAng_meas = measurements[3]
        snr_meas = measurements[4]

        #print('\tRANGE',range_meas,'DOPPLER:',doppler_meas,'azimuthAng:',azimuthAng_meas,'elevationAng:',elevationAng_meas, 'SNR:', snr_meas)
     #-----------------------------------------------

        #JSON serialise
        #print(json.dumps(packet.serialise()))

######################### Vehicle INIT & Connect #############################


print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

######################### MISSION DOWNLOAD & SAVE ############################

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format a
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)

def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip()) 


def get_wpL_length(wpL):
    return(len(wpL))      


#Download mission we just uploaded and save to a file
export_mission_filename = 'exportedmission001.txt'
save_mission(export_mission_filename)

printfile(export_mission_filename)

#Get the last waypoint number
wpList = download_mission()
last_wp = get_wpL_length(wpList)


mission_start_input = input("Mission set, Ready to takeoff and start: [Y/n]")

############ MISSION Functions ####################

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def countdown(t):
    
    while t:
        mins, secs = divmod(t, 60)
        timer = '{:02d}:{:02d}'.format(mins, secs)
        print('RTL in: %s (^c to cancel)' % (timer))
        time.sleep(1)
        t -= 1


"""/** Main program entry point */"""
if __name__ == '__main__':
    signal(SIGINT, signalHandler_SIGINT)

    srpControlClient.on('statechange', onStateChange)
    srpControlClient.start()

    srpDataClient.on('receive', onReceivePacket)

    if mission_start_input == "Y":
        print("Starting mission")

        # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
        arm_and_takeoff(10)

        # Set mode to AUTO to start mission
        vehicle.mode = VehicleMode("AUTO")

        while True:
            nextwaypoint=vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

            if (nextwaypoint == last_wp and distance_to_current_waypoint() < 2.0):
                print('Final waypoint reached, Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
                countdown(30)
                break

            time.sleep(1)

        print('Return to launch activated')
        vehicle.mode = VehicleMode("RTL")

        #Close vehicle object before exiting script
        print("Close vehicle object")
        vehicle.close()

    else: print("Exit...")

    while app.running:
        Sleep.ms(10)

    # --- shutdown and cleanup
    srpDataClient.stop()
    srpControlClient.stop()
