'''
This class uses DYNAMICALLY generated paths: all paths are generated
as the drone flies
'''

'''
File with the Tello_drone class which represents tello drone objects
and saves additional information about them such as their 
location, speed, waypoints, etc. while also including the implementation
of how and where they should move
'''

from djitellopy import tello 
import json
import math
import time

class Tello_drone:

    # NOTE: COORDINATES ARE MEASURED IN CM
    def __init__(self, i_x: int = 0, i_y: int = 0, i_z: int = 0, linear_speed: float = 16.0, angular_speed: float = 36.0, interval: float = 0.25, yaw = 0):

        # drone
        self.drone = tello.Tello()
        self.drone.connect()

        # coordinates
        self.x = i_x
        self.y = i_y
        self.z = i_z

        # current angle (in radians) the drone is traveling in
        self.angle = None

        # list of waypoints the drone must fly to
        # includes the start as the first waypoint
        self.waypoints = [(self.x, self.y, self.z)]

        # the current waypoint the drone is on
        self.current_waypoint = 0

        # the total distance to the next waypoint
        self.current_path_distance = None

        # the distance the drone has traveled on the current path
        self.current_path_traveled = 0

        # linear speed of the drone 
        self.lspeed = linear_speed

        # angular speed of the drone 
        self.aspeed = angular_speed

        # interval between commands
        self.interval = interval

        self.yaw = yaw


    def takeoff(self):
        # connects the drone and makes it takeoff
        # self.drone = tello.Tello()
        # self.drone.connect()
        self.drone.takeoff()
        #self.z += 100
    

    def land(self):
        # lands the drone
        self.drone.land()
        self.z = 0
    

    def add_waypoints_json(self, waypoint_file: str):
        # adds all of the waypoints in the json file as waypoints the drone must visit

        # Opening JSON file
        f = open(waypoint_file)
        
        # returns JSON object as 
        # a dictionary
        data = json.load(f)
        
        # Iterating through the json
        # list and adds them as waypoints
        # that the drone must visit
        for waypoint in data['wp']:
            new_waypoint = (waypoint["x"],waypoint["y"],waypoint["z"])

            self.waypoints.append(new_waypoint)
        
        # Closing file
        f.close()

        # FOR LATER
        # FUNCTION THAT WILL SORT WAYPOINTS IN A MATTER
        # SUCH THAT THE LEAST AMOUNT OF DISTANCE IS 
        # TRAVELED
        # self.sort_waypoints()
    

    def add_waypoints_list(self, waypoint_list):
        # adds all of the waypoints from the given list
        for waypoint in waypoint_list:
            new_waypoint = (waypoint[0],waypoint[1],waypoint[2])

            self.waypoints.append(new_waypoint)
        
        # FOR LATER
        # FUNCTION THAT WILL SORT WAYPOINTS IN A MATTER
        # SUCH THAT THE LEAST AMOUNT OF DISTANCE IS 
        # TRAVELED
        # self.sort_waypoints()
    



    # For later, will sort the waypoints such that the length of the path
    # the drone must take is minimized
    def sort_waypoints(self):
        pass

    
    # Private helper calculation functions
    def _calc_dist_btwn_wp(self, pos0, pos1) -> int:
        '''
        Private helper function that calculates the distance between 
        waypoints
        '''
        x = abs(pos0[0] - pos1[0])
        y = abs(pos0[1] - pos1[1])
        z = abs(pos0[2] - pos1[2])
        return int(math.hypot(x, y, z))


    def _calc_angle_btwn_wp_xy(self, pos0, pos1) -> int:
        '''
        Private helper function that calculates the angle between 
        waypoints
        Angle is calculated with respect to a reference point
        (posref)
        NOTE: using dot product calculation.
        '''

        # dummy coordinate along the current line the drone is on
        # used to calculate angle
        dummy_coordinate = (pos0[0]+10, pos0[1], pos0[2])


        # Calculates the magnitude of the angle
        ax = pos0[0] - dummy_coordinate[0]
        ay = pos0[1] - dummy_coordinate[1]
        bx = pos0[0] - pos1[0]
        by = pos0[1] - pos1[1]

        # Get dot product of pos0 and pos1.
        _dot = (ax * bx) + (ay * by)

        # Get magnitude of pos0 and pos1.
        _magA = math.sqrt(ax**2 + ay**2)
        _magB = math.sqrt(bx**2 + by**2)
        r_angle_rad = math.acos(_dot / (_magA * _magB))

        # Makes sure the angles are correct
        if pos0[1] >= pos1[1]:
            if pos0[0] > pos1[0]:
                r_angle_rad = 2*math.pi - r_angle_rad
            
            elif pos0[0] <= pos1[0]:
                r_angle_rad *= -1

        return r_angle_rad
    

    def move(self):
        '''
        Moves the drone along the current path
        '''

        # distance traveled with respect to linear speed and our interval
        dInterval = self.lspeed * self.interval

        # if we reach the next waypoint
        if len(self.waypoints) >= 2:
            if self.current_path_distance == None or self.current_path_traveled >= self.current_path_distance:

                if self.current_waypoint == 1:
                    self.drone.land()

                time.sleep(2)

                # updating which path we are on
                if self.current_waypoint + 1 >= len(self.waypoints): self.current_waypoint = 0
                else: self.current_waypoint += 1

                # calculate the distance to the next waypoint
                self.current_path_distance = self._calc_dist_btwn_wp(
                    self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[len(self.waypoints) - 1],
                    self.waypoints[self.current_waypoint]
                )

                # calculate the angle we must travel to the next waypoint
                self.angle = self._calc_angle_btwn_wp_xy(
                    self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[len(self.waypoints) - 1],
                    self.waypoints[self.current_waypoint]
                )

                self.current_path_traveled = 0

            time.sleep(self.interval)
            self.angle += self.yaw

            self.x += math.cos(self.angle) * dInterval
            self.y += math.sin(self.angle) * dInterval
            self.current_path_traveled += dInterval
            self.drone.send_rc_control(round(math.cos(self.angle) * self.lspeed), round(math.sin(self.angle) * self.lspeed), 0 ,0)

        

    # Get functions
    def get_waypoints(self):
        # Returns all of the drone's waypoints
        return self.waypoints
    

    def get_current_position(self):
        # Returns the current x y z position of the drone
        return (self.x, self.y, self.z)
    

    def get_speed(self):
        # Returns the speed of the drone
        return self.lspeed
        

                        

