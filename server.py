import socket
import os
import re
import sys
import random
import numpy as np
import math

MAX_NUMBER_OF_VALUES = 30
THRESHOLD = 2
valueTable = [] #[[ADDRESS[0], ADDRESS[1], VALUESLIST, INDEX_LAST_ADDED_VALUE],...]

"""
Least-square solving code taken from the tp of the course LINFO1113 - Algorithmique numérique

Choleski factorisation for matrix
"""
def choleski(a):
    n = len(a)
    for k in range(n):
        try:
            a[k,k] = math.sqrt(a[k,k] - np.dot(a[k,0:k],a[k,0:k]))
        except ValueError:
            raise Exception('Matrix is not positive definite')
        for i in range(k+1,n):
            a[i,k] = (a[i,k] - np.dot(a[i,0:k],a[k,0:k]))/a[k,k]
    for k in range(1,n): a[0:k,k] = 0.0 #erase upper triangle
    return a

def choleskiSol(L,b):
    n = len(b)
  # Solution of [L]{y} = {b}
    for k in range(n):
        b[k] = (b[k] - np.dot(L[k,0:k],b[0:k]))/L[k,k]
  # Solution of [L_transpose]{x} = {y}
    for k in range(n-1,-1,-1):
        b[k] = (b[k] - np.dot(L[k+1:n,k],b[k+1:n]))/L[k,k]
    return b

"""
Polynomial fit implementation
"""
def polyFit(xData, yData, m):
    a = np.zeros((m+1, m+1))
    b = np.zeros(m+1)
    s = np.zeros(2*m+1)
    for i in range(len(xData)):
        temp = yData[i]
        for j in range(m+1):
            b[j] += temp
            temp *= xData[i]
        temp = 1.0
        for j in range(2*m+1):
            s[j] += temp
            temp *= xData[i]
    for i in range(m+1):
        for j in range(m+1):
            a[i, j] = s[i+j]
    return choleskiSol(choleski(a), b)

def least_square(ytable, beginIndex):
    xtable = np.concatenate((np.arange(beginIndex, MAX_NUMBER_OF_VALUES, 1.), np.arange(0., beginIndex, 1.)))
    ret = polyFit(xtable, ytable, 1)
    if(ret[1] > THRESHOLD):
        return 1
    return 0

def add_new_data(address1, address2, value):
    for i in valueTable:
        if (i[0]==address1 and i[1]==address2):
            #print("size of the value table = " + str(len(i[2])))
            i[3] = (i[3] + 1) % MAX_NUMBER_OF_VALUES
            if (len(i[2]) == MAX_NUMBER_OF_VALUES):
                i[2][i[3]] = value
                return least_square(i[2], (i[3]+1) % MAX_NUMBER_OF_VALUES)
            else:
                i[2].append(value)
                if(len(i[2]) == MAX_NUMBER_OF_VALUES):
                    return least_square(i[2], (i[3]+1) % MAX_NUMBER_OF_VALUES)
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
