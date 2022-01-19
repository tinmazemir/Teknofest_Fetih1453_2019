import cv2
import numpy as np
import serial
import time

try:
    ser = serial.Serial("/dev/ttyACM0", 115200)
except:
    ser = serial.Serial("/dev/ttyACM1", 115200)

counter = 0

#Her rengin alt ve ust araliklari belirlenir
#
lowGreen = np.array((0, 130, 105))
highGreen = np.array((179, 255, 255))

#Blue HSV = (203, 100, 53)
lowRed = np.array((0, 120, 134))
highRed = np.array((13, 255, 255))

#Red HSV = (5, 92, 74)
lowBlue = np.array((89, 106, 6))
highBlue = np.array((168, 255, 255))
#===========================================

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
startBool = 1
def sendData(x, y, w, h, color):
    objectX = int((x + w)/2)
    objectY = int((y + h)/2)
    objectX = str(objectX)
    objectY = str(objectY)
    if(len(objectX) == 1):
        objectX = ("00" + objectX)
    elif(len(objectX) == 2):
        objectX = ("0" + objectX)

    if(len(objectY) == 1):
        objectY = ("00" + objectY)
    elif(len(objectY) == 2):
        objectY = ("0" + objectY)

    if(color == "green"):
        data = ("g" + objectX + "," + objectY)
    elif(color == "red"):
        data = ("r" + objectX + "," + objectY)
    elif(color == "blue"):
        data = ("b" + objectX + "," + objectY)
    ser.write(data.encode())
    #print(data)
def drawContours(contours, color):
    objectFound = 0
    largestContour = 0
    for index, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > largestContour:
            largestContour = area
            largestContour_index = index
            objectFound = 1

    if(objectFound and largestContour > 500):
        x,y,w,h = cv2.boundingRect(contours[largestContour_index])
        if(color == "green"):
            color = (0, 255, 0)
        elif(color == "blue"):
            color = (255, 0, 0)
        elif(color == "red"):
            color = (0, 0, 255)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 5, 0)
        return x, y, w, h
    return 0, 0, 0, 0

def startSignal():
    for _ in range(10):
      ser.write("s".encode())

while(True):
    ret, frame = cap.read()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if ret:
        img=cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        greenMask = cv2.inRange(hsv, lowGreen, highGreen)
        redMask = cv2.inRange(hsv, lowRed, highRed)
        blueMask = cv2.inRange(hsv, lowBlue, highBlue)

        greenContours = cv2.findContours(greenMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        redContours = cv2.findContours(redMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        blueContours = cv2.findContours(blueMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        gx, gy, gw, gh = drawContours(greenContours, "green")
        rx, ry, rw, rh = drawContours(redContours, "red")
        bx, by, bw, bh = drawContours(blueContours, "blue")
        if(counter == 10):
            sendData(rx, ry, rw, rh, "red")
            sendData(bx, by, bw, bh, "blue")
            sendData(gx, gy, gw, gh, "green")
            counter = 0
        cv2.imshow('1', frame)
        counter += 1


cap.release()
cv2.destroyAllWindows()
