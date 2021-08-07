import argparse
import time
from enum import Enum

import numpy as np
import copy

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class Phases(Enum):
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
        self.waypoint_index = 0 
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = Phases.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == Phases.TAKEOFF:
            if (-(self.local_position[2]) > 0.95 * self.target_position[2]):
                first_waypoint = self.local_position
                first_waypoint[2] = 3
                second_waypoint = copy.deepcopy(first_waypoint)
                second_waypoint[0] += 5 
                third_waypoint = copy.deepcopy(second_waypoint)
                third_waypoint[1] += 5
                fourth_waypoint = copy.deepcopy(third_waypoint)
                fourth_waypoint[0] -= 5
                self.all_waypoints.append(second_waypoint)
                self.all_waypoints.append(third_waypoint)
                self.all_waypoints.append(fourth_waypoint)
                self.all_waypoints.append(first_waypoint)
                print(self.all_waypoints)
                self.waypoint_transition()
        if self.flight_state == Phases.WAYPOINT:
            if (np.abs(self.local_position[0] - self.target_position[0])<0.5 and 
                np.abs(self.local_position[1] - self.target_position[1])<0.5 and
                np.abs(self.local_position[2] + self.target_position[2])<0.5):
                if self.waypoint_index == len(self.all_waypoints):
                    self.landing_transition()
                else:
                    print(self.waypoint_index)
                    self.target_position = self.all_waypoints[self.waypoint_index]
                    self.cmd_position(self.target_position[0],
                         self.target_position[1],
                         self.target_position[2],0)
                    self.waypoint_index += 1
        

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == Phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] <0.1)
            and (abs(self.local_position[2]) < 0.01)):
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_state == Phases.ARMING:
            self.takeoff_transition()
        elif self.flight_state == Phases.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        pass

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.flight_state = Phases.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.set_home_position(self.global_position[0],
                                self.global_position[1],
                                self.global_position[2])
        altitude = 3.0
        self.target_position[2] = altitude
        self.takeoff(altitude)
        self.flight_state = Phases.TAKEOFF
        print(self.flight_state)


    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.target_position = self.all_waypoints[0]
        self.waypoint_index += 1
        self.cmd_position(self.target_position[0],
                         self.target_position[1],
                         self.target_position[2],0)
        self.flight_state = Phases.WAYPOINT


    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = Phases.LANDING


    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = Phases.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = Phases.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
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
