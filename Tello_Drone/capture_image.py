'''
Created on Nov 22, 2021

@author: fangqiliu
'''

from djitellopy import tello
from time import sleep 
import cv2
from datetime import datetime,timezone



dt = datetime.now().replace(tzinfo=timezone.utc)

#datetime.strftime(datetime.now(), '%Y-%m-%d')
print(dt.isoformat())


me=tello.Tello()
me.connect()
print(me.get_battery())
 
me.streamon()
 
while True: 
    img= me.get_frame_read().frame
    #img=cv2.resize(img, (360,240))
    cv2.imshow("Image", img)
    cv2.waitKey(1)
