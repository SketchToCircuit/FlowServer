from base64 import b64decode
from re import X
import sys
import math
from tkinter import Y
from typing import List
from webbrowser import get
import cv2 as cv
import numpy as np
import json
from jsonc_parser.parser import JsoncParser
import numpy as np
class Linepoint:
    def __init__(self, x, y):
        self.X = x
        self.Y = y
class Line:
    def __init__(self, p1 : Linepoint, p2 : Linepoint):
        self.p1 = p1
        self.p2 = p2

def getCompPins(neuralOut):
    compPins = []
    for component in neuralOut:
        for pin in component["pins"]:
            compPins.append((
                pin["x"],
                pin["y"]
            ))
    return compPins

def getAngle(l1, l2 ):
    len1 = math.sqrt((l1[0] - l1[2])**2 + (l1[1] - l1[3])**2)
    len2 = math.sqrt((l2[0] - l2[2])**2 + (l2[1] - l2[3])**2)

    v1 = np.array((
        l1[0] - l1[2],
        l1[1] - l1[3]
    ))

    v2 = np.array((
        l2[0] - l2[2],
        l2[1] - l2[3]
    ))
    return math.acos((v1 @ v2) / (len1 * len2))

def findNearestLine(lines, point):
    nearestlines = []
    HITRADIUS = 100#px
    shortlines = []
    for i,line in enumerate(lines):
        distance1 = int(math.sqrt((point[0] - line[0])**2 + (point[1] - line[1])**2))
        distance2 = int(math.sqrt((point[0] - line[2])**2 + (point[1] - line[3])**2))
        if(distance1 < HITRADIUS and distance2 < HITRADIUS):
            shortlines.append(i)
        elif(distance1 < HITRADIUS):
            nearestlines.append((i, 1))
        elif(distance2 < HITRADIUS):
            nearestlines.append((i, 2))

    return nearestlines, shortlines
    
def checkForPin(startPoint, compPins):
    HITRADIUS = 40#px
    for pins in compPins:
        distance = math.sqrt((startPoint[0] - pins[0])**2 + (startPoint[1] - pins[1])**2)
        if distance < HITRADIUS:
           return True


def followLine(lines , currPoint, compPins, turtleList, img):
    cv.circle(img, currPoint, 50, (0, 255, 0), 2)
    nearestline, shortlines = findNearestLine(lines, currPoint)
    currLine = [turtleList[-1][0], turtleList[-1][1], currPoint[0], currPoint[1]]
    
    if not nearestline:
        print("nothing found")
        return turtleList
    #check the angle for the lines
    for line in nearestline:
        angle = getAngle(currLine, lines[line[0]])
        #20deg deviaion is counted as straight
        if angle < math.pi - (math.pi/8):
            print("found Straight")
            print(lines[line[0]])
            if line[1] == 1:
                currPoint = (
                    lines[line[0]][0],
                    lines[line[0]][1]
                )
                cv.line(img, (lines[line[0]][0], lines[line[0]][1]), (lines[line[0]][2], lines[line[0]][3]), (0,255,0), 7, cv.LINE_AA)
            elif line[1] == 2:
                currPoint = (
                    lines[line[0]][2],
                    lines[line[0]][3]
                )
        elif angle > 1.4 and angle < 1.7:
            pass

    if checkForPin(currPoint, compPins):
        pass
    
def NetListExP(neuralOut):
    NetList = []
    
    for comp in neuralOut:
        #gets topleft/bottomright cordinates
        topleft = [comp["topleft"]["x"], comp["topleft"]["y"]]
        botright = [comp["bottomright"]["x"], comp["bottomright"]["y"]]

        pins = []
        #iterates through the pins since thier amount is variable
        for pincoll in comp["pins"]:
            pins.append(
                {
                    "x": pincoll["x"], 
                    "y":  pincoll["y"],
                    "id": 0
                })
        #fill in the rest of the informations
        NetList.append({
            "component": comp["component"],
            "position": { 
                #center the component
                "x": topleft[0] + (abs(topleft[0] - botright[0]) / 2),
                "y": topleft[1] + (abs(topleft[1] - botright[1]) / 2)},
            "pins": pins
            })

    return reorderPins(NetList)

def detect(img, neuralOut):
    cdstP = np.copy(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
    _, img = cv.threshold(img, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

    #skeletonize the picture
    img = cv.ximgproc.thinning(img)
    #cv.imshow("", thin) 

    #predicting lines 
    linesP = cv.HoughLinesP(img, 1, np.pi / 180, 13, None, 0, 15)

    lines = []
    for line in linesP:
        lines.append(line[0])

    turtleList = [(227,327)]
    compPins = getCompPins(neuralOut)
    followLine(lines, (257, 324), compPins, turtleList, cdstP)
        
    #draw predicted lines just for debuging
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv.LINE_AA)
            cv.circle(cdstP, (l[0], l[1]), 2, (255, 0, 0), -1)
            cv.circle(cdstP, (l[2], l[3]), 2, (255, 0, 0), -1)
    
    
    
    cv.imshow("closed Source", img)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.waitKey()

def reorderPins(netList):
    resultNetList = []

    for cmp in netList:
        if cmp['component'] in ['OPV', 'S3', 'NPN', 'PNP', 'MFET_N_D', 'MFET_N_E', 'MFET_P_D', 'MFET_P_E', 'JFET_N', 'JFET_P']:
            pins = cmp['pins']
            center = np.array([cmp['position']['x'], cmp['position']['y']]) * 0.5 + (np.array([pins[0]['x'], pins[0]['y']]) + np.array([pins[1]['x'], pins[1]['y']]) + np.array([pins[2]['x'], pins[2]['y']])) / 3.0 * 0.5

            minError = float('inf')
            finalPins = pins

            for i in range(3):
                single = np.array([pins[i]['x'], pins[i]['y']]) - center
                a = np.array([pins[(i + 1) % 3]['x'], pins[(i + 1) % 3]['y']]) - center
                b = np.array([pins[(i + 2) % 3]['x'], pins[(i + 2) % 3]['y']]) - center

                singleAngle = math.atan2(single[1], single[0])
                
                angleA = math.atan(math.tan(math.atan2(a[1], a[0]) - singleAngle))
                angleB = math.atan(math.tan(math.atan2(b[1], b[0]) - singleAngle))

                otherAngleError = abs(angleA + angleB)
                error = abs(math.sin(2.0 * singleAngle)) * math.pi / 4.0 + otherAngleError

                if error < minError:
                    minError = error

                    if cmp['component'] == 'OPV':
                        finalPins = [pins[(i + 1) % 3], pins[(i + 2) % 3], pins[i]]
                    else:
                        finalPins = [pins[i], pins[(i + 1) % 3], pins[(i + 2) % 3]]

            cmp['pins'] = finalPins
            resultNetList.append(cmp)
        else:
            resultNetList.append(cmp)
    
    return resultNetList

def main():
    neuralOutput = JsoncParser.parse_file('TestData/SampleNeuralOutput.jsonc')

    img = cv.imread("TestData/test.jpg", cv.IMREAD_GRAYSCALE)
    detect(img, neuralOutput)
    
    #netList = NetListExP(neuralOutput)

    #netList = reorderPins(netList)

    #opv = [cmp for cmp in netList if cmp['component'] == 'OPV']
    #npn = [cmp for cmp in netList if cmp['component'] == 'NPN']
    #print(opv)
    #print(npn)

if __name__ == "__main__":
    main()