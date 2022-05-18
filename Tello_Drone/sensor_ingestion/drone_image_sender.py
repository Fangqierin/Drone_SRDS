import requests
from time import sleep 
from datetime import datetime,timezone
import base64
import socket
import json
url = 'http://128.195.52.200:5000/observation/image'
points=[(0,0),(0,0)]
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect(('dhcp-v038-114.mobile.uci.edu', 1241))

#print(sys.path)
def upload(dt,loc,img):
    
    data={}
    data['arrival_time']=f"{dt}"
    data['location']=loc
    data['image']=f"{base64.b64encode(img.read())}"
    print(data)
    #try:
    r = requests.post(url, json = data)
    print(data)
    if r.json()["result"] != "success":
        print("Post failed!!\n")
    else:
        print(f"Post the drone's image:{r.json()}")
    #except:
       # data2=json.dumps(data)
       # print(len(data2))
       # s.send(bytes(data2,"utf-8"))
        #print(f"Transfer the image to an local machine\n")
    
    return

if __name__== '__main__':     
    print(f"start")
    file="2022-01-1022:07:21.303788+00:00;(0.0,0.5)m.png"
    img=open(file,'rb')
    dt = datetime.now().replace(tzinfo=timezone.utc)
    loc="(0.0,0.5)m"
    upload(dt,loc,img)