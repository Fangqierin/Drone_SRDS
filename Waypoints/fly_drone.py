
"""
Main file meant for execution that will run the program and
make the drones fly around the given waypoints
"""


from tello_class import Tello_drone
import cv2

if __name__ == "__main__":
    try:

        main_drone = Tello_drone(0, -50)

        running = True

        cRound = 0
        check = main_drone.add_waypoints_database(f"{cRound}")
        cRound = 1

        frame = main_drone.drone.get_frame_read().frame
        cv2.imshow("Image", frame)
        cv2.waitKey(15000)
        cv2.destroyAllWindows()

        main_drone.takeoff()

        main_drone.move(True)
        
        while running:
            
            if main_drone.get_current_waypoint() == 0:
                # main_drone.hover()
                main_drone.reset_waypoints()
                check = main_drone.add_waypoints_database(f"{cRound}")
                if check: cRound+=1

            main_drone.move(check)

    finally:
        main_drone.land()








