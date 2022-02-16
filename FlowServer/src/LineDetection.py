from base64 import b64decode
from re import X
import sys
import math
from tkinter import Y
from typing import List
import cv2 as cv
import numpy as np
import json
from jsonc_parser.parser import JsoncParser
import numpy as np
class Linepoint:
    X = 0
    Y = 0
class Line:
    point1 = Linepoint 
    point2 = Linepoint

def getCompPins(neuralOut):
    compPins = []
    for pins in neuralOut["pins"]:
        compPins.append((
            pins["x"],
            pins["y"]
        ))
    return compPins


def findNearestLine(lines , point : Linepoint):
    line : Line
    HITRADIUS = 40#px
    for line in lines:
        distance1 = int(math.sqrt((point.X + line.point1.X)**2 + (point.Y + line.point1.Y)**2))
        distance2 = int(math.sqrt((point.X + line.point2.X)**2 + (point.Y + line.point2.Y)**2))
        if(distance1 < HITRADIUS and distance2 < HITRADIUS):
            

    

def checkForPin(startPoint, compPins):
    HITRADIUS = 40#px
    for pins in compPins:
        distance = math.sqrt((startPoint[0] - pins[0])**2 + (startPoint[1] - pins[1])**2)
        if distance < HITRADIUS:
           return True


def followLine(netlines, lines, startPoint, net : List, compPins):
    nearestline, shortlines = findNearestLine(lines, startPoint)

    if not nearestline:
        return net

    if checkForPin(startPoint, compPins):
        net.append(
            startPoint,
            nearestline
        )

    for foundline in nearestline:
        if foundline[1] == 2:
            startPoint = (
                lines[2],
                lines[3]
            )
        elif foundline[1] == 1:
            startPoint = (
                lines[0],
                lines[1]
            )
        findNet(netlines, lines, startPoint, net, compPins)



    return 0
    
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
    _, img = cv.threshold(img,128, 255, cv.THRESH_BINARY_INV)

    
    _, img = cv.threshold(img,128, 255, cv.THRESH_BINARY)
    rows = img.shape[0]

    #for now lines will be preped by whiteboxing all components
    for component in neuralOut:
        #set the width of the boundingbox to the points
        newTopleft = (
            component["topleft"]["x"],
            component["topleft"]["y"]         
        )
        newBotright = (
            component["bottomright"]["x"],
            component["bottomright"]["y"]
        )
        
        for pins in component["pins"]:
            newTopleft = (
                min(newTopleft[0], pins["x"]),
                component["topleft"]["y"]
            )
            newBotright = (
                max(newBotright[0], pins["x"]),
                component["bottomright"]["y"]
            )
        #reduce the height of the bounding box by 10%
        newTopleft = (
            int(newTopleft[0] * 1.05),
            int(newTopleft[1] * 1.05)
        )

        newBotright = (
            int(newBotright[0] * 0.95),
            int(newBotright[1] * 0.95)
        )
        
        cv.rectangle(img, newTopleft, newBotright, (0,0,0), -1)
        cv.rectangle(cdstP, newTopleft, newBotright, (0,0,255), 2)

    #cleaning away little lines after croping components
    kernel = np.ones((5,5),np.uint8)
    img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)   

    #predicting lines 
    linesP = cv.HoughLinesP(img, 1, np.pi / 180, 25, 5, 5)
    newLines = []
    for lines in linesP:
        bufferline = Line(Linepoint(lines[0], lines[1]), Linepoint(lines[2], lines[3]))
        newLines.append(bufferline)

    #draw predicted lines just for debuging
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv.LINE_AA)
    print(len(linesP))
    
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