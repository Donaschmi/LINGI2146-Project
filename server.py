import socket
import os
import re
import sys

def main():
    mysocket = socket.socket()
    mysocket.connect(('localhost',60001))
    while(True):
        data = mysocket.recv(4096)
        if not data:
            break
        print("data sent by border-router : " + repr(data))
    
    mysocket.close()

if __name__ == "__main__":
    main()