import socket

HEADERSIZE = 10

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), 1241))
print(f"see socketname {socket.gethostname()}")
s.listen(5)

while True:
    # now our endpoint knows about the OTHER endpoint.
    clientsocket, address = s.accept()
    print(f"Connection from {address} has been established.")

    msg = "Welcome to the server!"
    msg = f"{len(msg):<{HEADERSIZE}}"+msg

    clientsocket.send(bytes(msg,"utf-8"))       