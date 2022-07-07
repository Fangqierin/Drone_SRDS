
'''
This file houses the functions that will draw the
grid for us as well as the other functionality of
our grid GUI
'''

TESTING = False

from cv2 import inpaint
import pygame
import math
import time


# class that represents an input box
class InputBox:
    def __init__(
        self, screen, x: int = 10, y: int = 10, height: int = 35, width: int = 200, 
        input_string: str = "Add waypoint", active_color = "LIGHT BLUE", passive_color = "LIGHT GREY"):

        self.x = x
        self.y = y

        self.height = height
        self.width = width

        self.input_string = input_string
        self.input_rect = pygame.Rect(self.x, self.y, self.width, self.height)

        self.screen = screen
        self.active_color = active_color
        self.passive_color = passive_color

        self.pressed = False
    

    def draw(self):

        if self.pressed:
            pygame.draw.rect(self.screen, self.active_color, self.input_rect)
        else: 
            pygame.draw.rect(self.screen, self.passive_color, self.input_rect)

        text_surface = pygame.font.SysFont("Times New Roman", int(self.height/2)).render(self.input_string , True, (0, 0, 0))

        self.screen.blit(text_surface, (self.x+6, self.y+6))
    

    def tick(self, events) -> str:

        for event in events:
            
            # If user clicked on input box
            if event.type == pygame.MOUSEBUTTONDOWN: 
                if self.input_rect.collidepoint(event.pos):
                    self.pressed = True
                    if self.input_string == "Add waypoint":
                        self.input_string = ""
                else:
                    self.pressed = False
                    self.input_string = "Add waypoint"

            # if user is typing into input box
            if self.pressed and event.type == pygame.KEYDOWN:

                # Check for enter, if pressed return the input
                if event.key == pygame.K_RETURN:
                    r_string = self.input_string
                    self.input_string = ''
                    return r_string

                # Check for backspace
                elif event.key == pygame.K_BACKSPACE:
    
                    # get text input from 0 to -1 i.e. end.
                    self.input_string = self.input_string[:-1]
    
                # Unicode standard is used for string
                # formation
                else:
                    self.input_string += event.unicode
        return ''




# grid class
class Grid:

    def __init__(self, drone, scale_factor: int = 3):
        
        # pygame
        pygame.init()
        self.screen = pygame.display.set_mode((1400, 750))
        self.screen.fill((255,255,255))

        # Variable that controls the scale between cm and pixels
        # A scale factor of 1 means every cm = 1 px
        # A scale factor of 2 means every cm = 2 px
        self.scale_factor = scale_factor

        # Drone that occupies the grid
        self.drone = drone
        if not TESTING: self.drone.takeoff()

        # input box
        self.input_box = InputBox(self.screen)        



    def plot_points_rel_to_center(self, coordinates, color: str, is_drone: bool):
        '''
        Plots the given coordinates in the grid given how large the grid is
        NOTE: This function plots the points assuming the coordinates are oriented
        such that the center of the grid is 0,0
        '''

        # center coordinates of the grid
        center_x, center_y = self.screen.get_width()/2, self.screen.get_height()/2

        coord_num = 1
        # plot each coordinate relative to the center
        for coordinate in coordinates:
            pygame.draw.circle(
                self.screen, color, 
                (center_x + coordinate[0] * self.scale_factor, 
                center_y - coordinate[1] * self.scale_factor),
                5)
            
            if not is_drone:
                text_surface = pygame.font.SysFont("Times New Roman", 20).render(
                    f"{coord_num}: ({coordinate[0]}, {coordinate[1]}, {coordinate[2]})", True, (0, 0, 0))

                self.screen.blit(text_surface, 
                    (center_x + coordinate[0] * self.scale_factor + 4, 
                    center_y - coordinate[1] * self.scale_factor + 4))
            else:
                text_surface = pygame.font.SysFont("Times New Roman", 20).render(
                    f"({coordinate[0]}, {coordinate[1]}, {coordinate[2]})", True, (0, 0, 0))

                self.screen.blit(text_surface, 
                    (center_x + coordinate[0] * self.scale_factor + 4, 
                    center_y - coordinate[1] * self.scale_factor - 25))

            coord_num += 1

        

    def plot_grid_lines_rel_to_center(self, color: str, speed_scale: float = .5):
        '''
        Plots the grid lines with the given color and with
        respect to the drone's speed
        Variable speed_scale is how large a grid should be with respect to the drone's speed
        '''

        width, height = self.screen.get_size()

        # grid size (in pixels)
        px_grid_size = self.scale_factor * speed_scale * self.drone.get_speed()


        # Draw lines at the center outward
        center_x = width/2
        center_y = height/2

        pygame.draw.line(
                self.screen,
                color,
                (center_x, 0), (center_x, height))

        pygame.draw.line(
                self.screen,
                color,
                (0, center_y), (width, center_y))

        for column in range(1, math.ceil(width/(2*px_grid_size))):
            pygame.draw.line(
                self.screen,
                color,
                (center_x + column * px_grid_size, 0), (center_x + column * px_grid_size, height))
            
            pygame.draw.line(
                self.screen,
                color,
                (center_x - column * px_grid_size, 0), (center_x - column * px_grid_size, height))
        
        for row in range(1, math.ceil(height/(2*px_grid_size))):
            pygame.draw.line(
                self.screen,
                color,
                (0, center_y + row * px_grid_size), (width, center_y + row * px_grid_size))
            
            pygame.draw.line(
                self.screen,
                color,
                (0, center_y - row * px_grid_size), (width, center_y - row * px_grid_size))
    

    def plot_path(self, color: str = "ORANGE"):
        '''
        Plots the path of the drone
        '''

        center_x, center_y = self.screen.get_width()/2, self.screen.get_height()/2
        waypoints = self.drone.get_waypoints()

        for i in range(1,len(waypoints)):

            pygame.draw.line(
                self.screen,
                color,
                (center_x + waypoints[i-1][0] * self.scale_factor, center_y - waypoints[i-1][1] * self.scale_factor), 
                (center_x + waypoints[i][0] * self.scale_factor, center_y - waypoints[i][1] * self.scale_factor),
                3)
                    
        # Path from last waypoint to first
        pygame.draw.line(
                self.screen,
                color,
                (center_x + waypoints[-1][0] * self.scale_factor, center_y - waypoints[-1][1] * self.scale_factor), 
                (center_x + waypoints[0][0] * self.scale_factor, center_y - waypoints[0][0] * self.scale_factor),
                3)

    

    def tick(self) -> bool:
        '''
        One tick of the grid simulation
        '''

        self.screen.fill((255,255,255))

        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                if not TESTING: self.drone.land()
                return False

        new_waypoint = self.input_box.tick(events)
        if new_waypoint != '':
            self.drone.add_waypoints_list([tuple(map(int, new_waypoint.split(',')))])

        # moves the drone
        if not TESTING: self.drone.move()


        # plot the gird
        self.plot_grid_lines_rel_to_center("GREY")


        # draw the input box
        self.input_box.draw()

        # draw the path
        self.plot_path()

        # Plot the waypoints onto the grid
        self.plot_points_rel_to_center(self.drone.get_waypoints(), "BLUE", False)

        # Plot the drone onto the grid
        self.plot_points_rel_to_center([self.drone.get_current_position()], "BLACK",  True)

        pygame.display.update()

        return True
    




