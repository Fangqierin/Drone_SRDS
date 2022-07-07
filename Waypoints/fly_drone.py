
"""
Main file meant for execution that will run the program and
make the drones fly around the given waypoints
"""


from tello_class_dynamic_paths import Tello_drone
#import json
from grid import Grid


if __name__ == "__main__":

    # connect to drone and initial takeoff
    # NOTE: COORDINATES ARE MEASURED IN CM
    main_drone = Tello_drone()

    # add the hard coded waypoints to our main drone
    main_drone.add_waypoints_json("waypoints.json")


    # flight loop
    running = True


    # grid
    grid = Grid(main_drone)

    while running:

        running = grid.tick()






