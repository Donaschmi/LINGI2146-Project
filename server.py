import socket
import os
import re
import sys
import random
import numpy as np





MAX_NUMBER_OF_VALUES = 5
THRESHOLD = 2
valueTable = [] #[[ADDRESS[0], ADDRESS[1], VALUESLIST, INDEX_LAST_ADDED_VALUE],...]

def least_square(ytable):
    x = np.linspace(start=0,stop=MAX_NUMBER_OF_VALUES-1,num=MAX_NUMBER_OF_VALUES)
    matrix = [x,]*len(ytable)
    ret = np.linalg.lstsq(matrix,ytable)
    print(ret[0][1])
    if(ret[0][1]>THRESHOLD):
        return 1
    return 1

def add_new_data(address1, address2, value):
    for i in valueTable:
        if (i[0]==address1 and i[1]==address2):
            #print("size of the value table = " + str(len(i[2])))
            i[3] = (i[3] + 1) % MAX_NUMBER_OF_VALUES
            if (len(i[2]) == MAX_NUMBER_OF_VALUES):
                i[2][i[3]] = value
                return least_square(i[2])
            else:
                i[2].append(value)
                if(len(i[2]) == MAX_NUMBER_OF_VALUES):
                    return least_square(i[2])
                return 0
    valueTable.append([address1, address2, [value], 0])
    return 0




def main():
    mysocket = socket.socket()
    mysocket.connect(('localhost',60001))
    string = ""
    counter=0
    while(True):
        data = mysocket.recv(1)

        if data.decode("utf-8")=="\n":
            splitted = string.split()
            if splitted[0] == "data":   #data 'format' = "data ADDRESS[0] ADDRESS[1] SENSOR_VALUE"
                print("Received data from border-router : " + str(splitted[1:]))
                openvalve = add_new_data(int(splitted[1]), int(splitted[2]), float(splitted[3]))
                if openvalve==1 :
                    print("OPEN THE VALVES FOR " + splitted[1] + "." + splitted[2] + " !")
                    mysocket.send(("open "+ splitted[1] + " " + splitted[2] + " \n").encode("utf-8"))
            string=""
            counter+=1
        else:
            string+=data.decode("utf-8")

    mysocket.close()

if __name__ == "__main__":
    main()
