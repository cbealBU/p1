import socket
import os

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind(("127.0.0.1",5005))
while True:
    data,addr = sock.recvfrom(1024)
    if data == '\xbb':
        print("Ignition off, shutting down")
        os.system("sudo shutdown -h now")
    elif data == '\x83':
        print("Stopping default SPI comm")
        os.system("sudo systemctl stop spiComm.service")