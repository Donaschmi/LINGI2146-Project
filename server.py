import socket
import os
import re
import sys

def main():
    mysocket = socket.socket()
    mysocket.connect(('localhost',60001))
    string = ""
    counter=0
    while(counter<10):
        data = mysocket.recv(1)

        if data.decode("utf-8")=="\n":
            print("data sent by border-router : " + string)
            string=""
            counter+=1
        else:
            string+=data.decode("utf-8")

    mysocket.close()

if __name__ == "__main__":
    main()
