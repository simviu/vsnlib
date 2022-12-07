import socket
import sys
import time

HOST, PORT = "localhost", 8192


# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if sock is None:
    print("Socket failed")
    exit(1)

print("Socket client connected to:"+HOST+":"+str(PORT))
# Connect to server and send data
sock.connect((HOST, PORT))

i = 0
while True:
    s = "hello " + str(i)
    i = i + 1
    print("send :'"+s+"'")
    sock.sendall(bytes(s, "utf-8"))

    # Receive data from the server and shut down
    s = str(sock.recv(1024), "utf-8")
    print("recv:"+s)

    time.sleep(1)
