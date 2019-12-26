import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID
from scipy.spatial import distance

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # Register the callbacks for the position, the velocity, and the state
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):

        if self.flight_state == States.TAKEOFF:
            #coordinate conversion
            altitude = -1.0 * self.local_position[2]
            #check if altitude is within 95% of target
            if altitude > 0.95 *self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
                
                
        if self.flight_state == States.WAYPOINT:
            dst = distance.euclidean(self.target_position[0:2], self.local_position[0:2])
            if dst < 0.1 :      # 0,1 meter precision between the local precison and the demanded target position         
                    # Case 1 : Stay in the "Waypoint" State to reach the new waypoint
                    if len(self.all_waypoints) > 0:  
                            self.waypoint_transition()
                    
                    # Case 2 : The route of the drone is finished
                    # Transition evaluation to go in the state "Landing"
                    else : self.landing_transition()
                

    def velocity_callback(self):
        # the drone must be less than 0,01 meters above the ground level
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] <0.1) and
            abs(self.local_position[2]) <0.01):
                    self.disarming_transition()


    def state_callback(self):
        #  guided and armed internal state verification before to make a transition to a new state
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed and self.guided:
                self.takeoff_transition() 
        elif self.flight_state == States.DISARMING:
            if not self.armed and self.guided:
                self.manual_transition()

    def calculate_box(self):     
        print("Square_Waypoints")
        # Return waypoints to fly a Square
        # North/East/Down/Heading
        Square_waypoints = [[0.0, 10.0, 5.0, 0.0], [10.0, 10.0, 5.0, 0.0], [10.0, 0.0, 5.0, 0.0], [0.0, 0.0, 5.0, 0.0]] 
        return Square_waypoints 


    def arming_transition(self):
        print("arming transition")
        # Take control of the drone
        self.take_control()
        # Pass an arming command
        self.arm()
        # Set the home location to current position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        # Transition to the Arming State
        self.flight_state = States.ARMING


    def takeoff_transition(self):
        print("takeoff transition")
        # Set target_position altitude to 5.0m
        self.target_position[2] = 5.0 
        # Command a takeoff to 5.0m
        self.takeoff(self.target_position[2])
        # Transition to the TAKEOFF state
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        # Command the next waypoint position
        self.target_position =  self.all_waypoints.pop(0) # remove the first waypoint of the list
        self.cmd_position( self.target_position [0], self.target_position[1],  self.target_position[2], self.target_position[3])
        # Transition to WAYPOINT state
        self.flight_state = States.WAYPOINT
        

    def landing_transition(self):
        print("landing transition")
        # Command the drone to land
        self.land()
        # Transition to the LANDING state
        self.flight_state = States.LANDING


    def disarming_transition(self):
        print("disarm transition")
        # Command the drone to disarm
        self.disarm()
        # Transition to the DISARMING state 
        self.flight_state = States.DISARMING
        
    def manual_transition(self):
        print("manual transition")
        # Release control of the drone
        self.release_control()
        # Stop the connection (and telemetry log)
        self.stop()
        # End the mission
        self.in_mission = False
        # Transition to the MANUAL state
        self.flight_state = States.MANUAL

    def start(self):
        # Open a log file
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        # Start the drone connection
        print("starting connection")
        self.connection.start()
        # Close the log file
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
