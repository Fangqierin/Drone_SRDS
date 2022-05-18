import socket
from datetime import datetime, timezone
HEADERSIZE = 10

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('dhcp-v038-114.mobile.uci.edu', 1241))

import json 
#while True:
full_msg = ''
new_msg = True
# while True:
     #msg = s.recv(16)
#         if new_msg:
#             print("new msg len:",msg[:HEADERSIZE])
#             msglen = int(msg[:HEADERSIZE])
#             new_msg = False
# 
#         print(f"full message length: {msglen}")
# 
#         full_msg += msg.decode("utf-8")
#         #print(len(full_msg))
#         if len(full_msg)-HEADERSIZE == msglen:
#             print("full msg recvd")
#             print(full_msg[HEADERSIZE:])
#             new_msg = True
dt = datetime.now().replace(tzinfo=timezone.utc)
data={}
data['arrival_time']=f"{dt}"
data['location']="test"#loc
data['image']="test"#"f"{base64.b64encode(img.read())}"
data2=json.dumps(data)
print(len(data2))
s.send(bytes(data2,"utf-8"))
       
        
        
        
        
        