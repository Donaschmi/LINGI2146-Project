import socket
import os
import re
import sys

def main():
    mysocket = socket.socket()
    mysocket.connect(('localhost',60001))
    string = ""
    counter=0
    while(True):
        data = mysocket.recv(1)

        if data.decode("utf-8")=="\n":
            splitted = string.split()
            if splitted[0] == "borderinfo":
                print("borderRouter received msg")
            else:
                print("data sent by border-router : " + string)
                mysocket.send("message recu !\n".encode("utf-8"))
            string=""
            counter+=1
        else:
            string+=data.decode("utf-8")

    mysocket.close()

if __name__ == "__main__":
    main()
