
'''
File with the Tello_drone class which represents tello drone objects
and saves additional information about them such as their 
location, speed, waypoints, etc. while also including the implementation
of how and where they should move
'''

'''
This class uses DYNAMICALLY generated paths: all paths are generated
as the drone flies
'''

'''
Limitations: 
1. All coordinates should be divisible by the speed * interval to reduce rounding as much
as possible
2. Drone will fly incorrectly at low speeds
3. Drone flies differently at different altitudes (higher altitudes = higher speed)
4. The speed of the program also affects drone flight
'''

from djitellopy import tello 
import json
import math
import time
import cv2
# import base64
# from PIL import Image
# import io

# from os import remove
from glob import glob
from pymongo import MongoClient
from image_processing import detect_fire
from image_processing import calc_location_fire

from mongo_dashboard_FQ import GRID_SIZE

client = MongoClient("mongodb://127.0.0.1:27017/")


class Tello_drone:

    # NOTE: COORDINATES ARE MEASURED IN CM
    def __init__(self, i_x: int = 0, i_y: int = 0, i_z: int = 90,):

        # drone
        self.drone = tello.Tello()
        self.drone.connect()

        self.drone.streamon()
        
        # coordinates
        self.x = i_x
        self.y = i_y

        # NOTE: z will be assumed by default to be at a height of 80 cm
        # because that is the takeoff altitude and we want the drone the
        # first waypoint to be its starting position after takeoff
        self.z = i_z

        # list of waypoints the drone must fly to
        # includes the start as the first waypoint
        self.waypoints = []

        # the current waypoint the drone is on
        self.current_waypoint = 0

        # # the total distance to the next waypoint in terms of left right foward backward
        # self.current_path_distance_xy = 0

        # # the total distance to the next waypoint in terms of up and down
        # self.current_path_distance_z = 0

        # # the distance the drone has traveled on the current path in terms of left right foward backward
        # self.current_path_traveled_xy = 0

        # # the distance the drone has traveled on the current path in terms of up and down
        # self.current_path_traveled_z = 0

        # # linear and vertical speed of the drone 
        # self.lspeed = linear_speed

        # # angular speed of the drone 
        # self.aspeed = angular_speed

        # # interval between commands
        # self.interval = interval

        # # how much distance the drone covers in left right foward backward up and down each interval
        # self.change = self.lspeed * self.interval

        # self.yaw = yaw


    def takeoff(self):
        # connects the drone and makes it takeoff

        # makes drone fly to designated start height
        self.z = 90
        self.drone.send_control_command("takeoff", timeout = 30)
        #self.drone.takeoff()

        # for _ in range(abs(round((initial_height - 90)/self.change))+1):
        #     self.move(False)

        # if initial_height > 90:
        #     self.drone.move_up(initial_height - 90)
        # elif initial_height < 90:
        #     self.drone.move_down(90 - initial_height)

        # waypoints cannot be empty
        self.waypoints.insert(0,(self.waypoints[0][0],self.waypoints[0][1],90))

        print(self.waypoints)

        self.move(False)

        self.waypoints.pop(0)
        self.current_waypoint = 0
    

    def land(self):
        # lands the drone
        self.z = 0
        self.drone.land()
        self.drone.streamoff()

    

    # def add_waypoints_json(self, waypoint_file: str):
    #     # adds all of the waypoints in the json file as waypoints the drone must visit

    #     # Opening JSON file
    #     f = open(waypoint_file)

    #     # returns JSON object as 
    #     # a list
    #     waypoints = json.load(f)

    #     # Iterating through the json
    #     # list and adds them as waypoints
    #     # that the drone must visit
    #     for waypoint in waypoints:
    #         new_waypoint = (waypoint[0],waypoint[1],waypoint[2])
    #         self.waypoints.append(new_waypoint)
        
    #     # Closing file
    #     f.close()

    #     # FOR LATER
    #     # FUNCTION THAT WILL SORT WAYPOINTS IN A MATTER
    #     # SUCH THAT THE LEAST AMOUNT OF DISTANCE IS 
    #     # TRAVELED
    #     # self.sort_waypoints()
    

    # def add_waypoints_list(self, waypoints):
    #     # adds all of the waypoints from the given list
    #     for waypoint in waypoints:
    #         new_waypoint = (waypoint[0],waypoint[1],waypoint[2])

    #         self.waypoints.append(new_waypoint)
                    
    #     # FOR LATER
    #     # FUNCTION THAT WILL SORT WAYPOINTS IN A MATTER
    #     # SUCH THAT THE LEAST AMOUNT OF DISTANCE IS 
    #     # TRAVELED
    #     # self.sort_waypoints()
    

    def add_waypoints_database(self, currRound):
        # adds all of the waypoints from the database (client)

        waypoints = list(client["waypoints"].currentWaypoints.find({"round": currRound}))

        for waypoint in waypoints:
            new_waypoint = (int(waypoint['x']),int(waypoint['y']),int(waypoint['z']))

            self.waypoints.append(new_waypoint)
        
        # The first element of the list is the dummy initial waypoint so we skip it
        if len(waypoints) == 0: return False

        self.waypoints.append(self.waypoints[0])

        return True
                    
        # FOR LATER
        # FUNCTION THAT WILL SORT WAYPOINTS IN A MATTER
        # SUCH THAT THE LEAST AMOUNT OF DISTANCE IS 
        # TRAVELED
        # self.sort_waypoints()
    

    def reset_waypoints(self):
        '''
        Resets waypoints but keeps the first one
        '''
        if len(self.waypoints) == 0: return
        self.waypoints = [self.waypoints[0]]


    # # For later, will sort the waypoints such that the length of the path
    # # the drone must take is minimized
    # def sort_waypoints(self):
    #     pass

    
    # # Private helper calculation functions
    # def _calc_dist_btwn_wp_xy(self, pos0, pos1) -> int:
    #     '''
    #     Private helper function that calculates the distance between 
    #     waypoints
    #     '''
    #     x = abs(pos0[0] - pos1[0])
    #     y = abs(pos0[1] - pos1[1])
    #     return int(math.hypot(x, y))


    # def _calc_angle_btwn_wp_xy(self, pos0, pos1) -> int:
    #     '''
    #     Private helper function that calculates the angle between 
    #     waypoints
    #     Angle is calculated with respect to a reference point
    #     (posref)
    #     NOTE: using dot product calculation.
    #     '''

    #     # dummy coordinate along the current line the drone is on
    #     # used to calculate angle
    #     dummy_coordinate = (pos0[0]+10, pos0[1], pos0[2])


    #     # Calculates the magnitude of the angle
    #     ax = pos0[0] - dummy_coordinate[0]
    #     ay = pos0[1] - dummy_coordinate[1]
    #     bx = pos0[0] - pos1[0]
    #     by = pos0[1] - pos1[1]

    #     # Get dot product of pos0 and pos1.
    #     _dot = (ax * bx) + (ay * by)

    #     # Get magnitude of pos0 and pos1.
    #     _magA = math.sqrt(ax**2 + ay**2)
    #     _magB = math.sqrt(bx**2 + by**2)
    #     r_angle_rad = math.acos(_dot / (_magA * _magB))

    #     # Makes sure the angles are correct
    #     if pos0[1] >= pos1[1]:
    #         if pos0[0] > pos1[0]:
    #             r_angle_rad = 2*math.pi - r_angle_rad
            
    #         elif pos0[0] <= pos1[0]:
    #             r_angle_rad *= -1

    #     return r_angle_rad
    
    
    # def move(self, take_picture: bool = True):
    #     '''
    #     Moves the drone along the current path
    #     '''

    #     '''
    #     NOTE: THIS IS NOT THE MOST EFFICIENT IMPLEMENTATION
    #     THIS IS SIMPLY THE EASIEST IMPLEMENTATION

    #     INSTEAD OF MOVING ALONG THE 3D VECTOR BETWEEN TWO POINTS, 
    #     DUE TO ROUNDING LIMITATIONS OUR DRONE WILL JUST MOVE SEPERATELY
    #     BETWEEN ITS X AND Y MOVEMENT VS ITS Z MOVEMENT INSTEAD OF MOVING
    #     WITH RESPECT TO ALL 3 AT THE SAME TIME

    #     IF THE DRONE REACHES ONE X AND Y BEFORE IT REACHES Z IT WILL JUST
    #     PURELY FLY IN THE Z DIRECTION TO CORRECT THIS AND VICE VERSA
    #     '''
        
    #     # if we reach the next waypoint
    #     if len(self.waypoints) >= 2:
            
    #         # Checks if the drone has flown far enough in terms of left right back and foward
    #         xy_check = self.current_path_traveled_xy >= self.current_path_distance_xy

    #         # Checks if the drone has flown far enough up and down
    #         z_check = (self.current_path_traveled_z >= abs(self.current_path_distance_z))

    #         if xy_check and z_check:

    #             time.sleep(5)

    #             if take_picture:
    #                 self.drone.send_rc_control(0,0,0,0)
    #                 self.upload_current_frame()


    #             # updating which path we are on
    #             if self.current_waypoint + 1 >= len(self.waypoints): self.current_waypoint = 0
    #             else: self.current_waypoint += 1

    #             # calculate the distance to the next waypoint in terms of left right fowards and backwards
    #             self.current_path_distance_xy = self._calc_dist_btwn_wp_xy(
    #                 self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1],
    #                 self.waypoints[self.current_waypoint]
    #             )

    #             # calculate the distance to the next waypoint in terms of up and down
    #             self.current_path_distance_z = self.waypoints[self.current_waypoint][2] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[2]


    #             # calculate the angle we must travel to the next waypoint
    #             if self.current_path_distance_xy != 0:
    #                 self.angle = self._calc_angle_btwn_wp_xy(
    #                     self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[- 1],
    #                     self.waypoints[self.current_waypoint]
    #                 )

    #             self.current_path_traveled_xy = 0
    #             self.current_path_traveled_z = 0

    #             if self.current_path_distance_xy != 0:
    #                 xy_check = False

    #             z_check = False

    #         else:
    #             time.sleep(self.interval)
    #             self.angle += self.yaw

    #             if not xy_check:
    #                 self.x += math.cos(self.angle) * self.change
    #                 self.y += math.sin(self.angle) * self.change

    #                 self.current_path_traveled_xy += self.change

    #                 self.drone.send_rc_control(
    #                 round(math.cos(self.angle) * self.lspeed * (1 if self.z < 140 else 7.0/8.0)) if not xy_check else 0, 
    #                 round(math.sin(self.angle) * self.lspeed * (1 if self.z < 140 else 7.0/8.0)) if not xy_check else 0, 
    #                 0,0)

                
    #             else:
    #                 self.z += self.change if self.current_path_distance_z >= 0 else -self.change
    #                 self.current_path_traveled_z += self.change

    #                 self.drone.send_rc_control(0,0, 
    #                 round(self.lspeed * 1.5 if self.current_path_distance_z >= 0 else -self.lspeed * 1.5) if not z_check else 0,0)
    


    # def move(self, take_picture: bool = True):
    #     '''
    #     Moves the drone along the current path
    #     '''

    #     '''
    #     NOTE: THIS IS NOT THE MOST EFFICIENT IMPLEMENTATION
    #     THIS IS SIMPLY THE EASIEST IMPLEMENTATION

    #     INSTEAD OF MOVING ALONG THE 3D VECTOR BETWEEN TWO POINTS, 
    #     DUE TO ROUNDING LIMITATIONS OUR DRONE WILL JUST MOVE SEPERATELY
    #     BETWEEN ITS X AND Y MOVEMENT VS ITS Z MOVEMENT INSTEAD OF MOVING
    #     WITH RESPECT TO ALL 3 AT THE SAME TIME

    #     IF THE DRONE REACHES ONE X AND Y BEFORE IT REACHES Z IT WILL JUST
    #     PURELY FLY IN THE Z DIRECTION TO CORRECT THIS AND VICE VERSA
    #     '''
        
    #     # if we reach the next waypoint
    #     if len(self.waypoints) >= 2:

    #         if take_picture:
    #             self.drone.send_rc_control(0,0,0,0)
    #             self.upload_current_frame()


    #         # updating which path we are on
    #         if self.current_waypoint + 1 >= len(self.waypoints): self.current_waypoint = 0
    #         else: self.current_waypoint += 1

    #         l_r_distance = self.waypoints[self.current_waypoint][0] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[0]
    #         b_f_distance = self.waypoints[self.current_waypoint][1] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[1]
    #         u_d_distance = self.waypoints[self.current_waypoint][2] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[2]

    #         if abs(l_r_distance) >= 20:
    #             self.drone.move('right' if l_r_distance >=0 else 'left', abs(l_r_distance))
    #             self.x += l_r_distance
    #         if abs(b_f_distance) >= 20:
    #             self.drone.move('forward' if b_f_distance >=0 else 'back', abs(b_f_distance))
    #             self.y += b_f_distance
    #         if abs(u_d_distance) >= 20:
    #             self.drone.move('up' if u_d_distance >=0 else 'down', abs(u_d_distance))
    #             self.z += u_d_distance
    

    def move(self, take_picture: bool = True):
        '''
        Moves the drone along the current path
        '''
        
        # if we reach the next waypoint
        if len(self.waypoints) >= 2:

            if take_picture:
                self.drone.send_rc_control(0,0,0,0)
                self.upload_current_frame()


            # updating which path we are on
            if self.current_waypoint + 1 >= len(self.waypoints): self.current_waypoint = 0
            else: self.current_waypoint += 1

            l_r_distance = self.waypoints[self.current_waypoint][0] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[0]
            b_f_distance = self.waypoints[self.current_waypoint][1] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[1]
            u_d_distance = self.waypoints[self.current_waypoint][2] - (self.waypoints[self.current_waypoint - 1] if self.current_waypoint != 0 else self.waypoints[-1])[2]

            if abs(l_r_distance) >= 20:
                self.drone.send_control_command("{} {}".format('right' if l_r_distance >=0 else 'left', abs(l_r_distance)),timeout = 40)
                self.x += l_r_distance
            elif abs(l_r_distance) > 0:
                for _ in range(l_r_distance//10):
                    self.drone.send_rc_control(10 * (1 if l_r_distance >=0 else -1),0,0,0)
                    time.sleep(1)
                self.drone.send_rc_control(0,0,0,0)
            self.x = self.waypoints[self.current_waypoint][0]

            if abs(b_f_distance) >= 20:
                self.drone.send_control_command("{} {}".format('forward' if b_f_distance >=0 else 'back', abs(b_f_distance)),timeout = 40)
            elif abs(b_f_distance) > 0:
                for _ in range(b_f_distance//10):
                    self.drone.send_rc_control(0,10 * (1 if l_r_distance >=0 else -1),0,0)
                    time.sleep(1)
                self.drone.send_rc_control(0,0,0,0)
            self.y = self.waypoints[self.current_waypoint][1]


            if abs(u_d_distance) >= 20:
                self.drone.send_control_command("{} {}".format('up' if u_d_distance >=0 else 'down', abs(u_d_distance)),timeout = 40)
            elif abs(u_d_distance) > 0:
                for _ in range(u_d_distance//10):
                    self.drone.send_rc_control(0,0,10 * (1 if l_r_distance >=0 else -1),0)
                    time.sleep(1)
                self.drone.send_rc_control(0,0,0,0)
            self.z = self.waypoints[self.current_waypoint][2]

    

    def hover(self):
        '''
        makes the drone hover in place
        '''
        self.drone.send_rc_control(0,0,0,0)
    

    # Streaming and Image Processing Functions
    def stream_current_frame(self):
        # Streams the current frame the drone's camera has captured
        frame = self.drone.get_frame_read().frame

        cv2.imshow("Stream", frame)
    

    def upload_current_frame(self):
        '''
        Uploads the current frame the drone's camera has captured
        with relevant meta data
        '''

        frame = self.drone.get_frame_read().frame

        # STORES IMAGE TO LOCAL DATABASE
        # WITH METADATA
        num_images = len(glob("./test_images/*"))

        frame = cv2.flip(frame, 0)

        # stores image locally
        cv2.imwrite(f'test_images/image_waypoint_{num_images}.png', frame)

        # stores path to image and other relevant metadata in the database
        client["images"].currentImages.insert_one(
            {
                "path": f'./test_images/image_waypoint_{num_images}.png',
                "location": (round(self.x),round(self.y),round(self.z)),
                "time": "to be implemented"
            }
        )
        self.image_processing(frame)



    # Get functions
    def get_waypoints(self):
        # Returns all of the drone's waypoints
        return self.waypoints
    

    def get_current_position(self):
        # Returns the current x y z position of the drone
        return (self.x, self.y, self.z)

    def get_current_waypoint(self):
        # Returns the current waypoint of the drone
        return self.current_waypoint
    
    def image_processing(self, image):
        '''
        triggers image processing for whenever an image is stored
        '''
        #print("trigger_image_processing called")

        #num_images = len(glob("./test_images_results/*"))


        fire_list = detect_fire(image)

        fire_list = calc_location_fire((self.x, self.y, self.z), fire_list)

        print()
        print(fire_list)
        print()

        firedb = client["fireMap"]
        gridCollection = firedb.gridStates

        for fire in fire_list:

            # top left
            tLx, tLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
            gridCollection.update_one({'location': (tLx,tLy)},{ '$set': {'state': 1.0}})

            # top right
            tRx, tRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
            gridCollection.update_one({'location': (tRx,tRy)},{ '$set': {'state': 1.0}})

            # bottom left
            bLx, bLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
            gridCollection.update_one({'location': (bLx,bLy)},{ '$set': {'state': 1.0}})

            # bottom right
            bRx, bRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
            gridCollection.update_one({'location': (bRx,bRy)},{ '$set': {'state': 1.0}})
            


    # def get_speed(self):
    #     # Returns the speed of the drone
    #     return self.lspeed

        

                        

