
"""
Main file meant for execution that will run the program and
make the drones fly around the given waypoints
"""


from tello_class import Tello_drone
from grid import Grid
import time
import pygame
import os
import asyncio
import cv2

if __name__ == "__main__":
    # The grid is turned off for now for performance
    # grid
    # grid = Grid(main_drone)
    try:
        # pygame.init()
        # screen = pygame.display.set_mode((900, 600))

        main_drone = Tello_drone(0, -50)

        running = True

        # main_drone.add_waypoints_json("waypoints.json")        
        cRound = 0
        check = main_drone.add_waypoints_database(f"{cRound}")
        cRound = 1

        frame = main_drone.drone.get_frame_read().frame
        cv2.imshow("Image", frame)
        cv2.waitKey(15000)
        cv2.destroyAllWindows()

        # screen = pygame.display.set_mode((900, 600))

        main_drone.takeoff()



        main_drone.move(True)
        

        while running:
            
            if main_drone.get_current_waypoint() == 0:
                # main_drone.hover()
                main_drone.reset_waypoints()
                check = main_drone.add_waypoints_database(f"{cRound}")
                if check: cRound+=1

            main_drone.move(check)

            # events = pygame.event.get()
            # for event in events:
            #     if event.type == pygame.QUIT:
            #         main_drone.drone.land()
            #         running = False


            #running = grid.tick()

    finally:
        main_drone.land()



# if __name__ == "__main__":
#     asyncio.run(main())







