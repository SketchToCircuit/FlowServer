from base64 import b64decode
from pickle import FALSE, TRUE
import sys
import math
import cv2 as cv
import numpy as np
import json
from jsonc_parser.parser import JsoncParser

def detectNet(lines, neuralOut):
    #lines left defines how many lines(pointsare left)
    HITRADIUS = 40 #px
    linesLeft = lines
    nets = []

    #create a list of all component pins
    pinPoints = []
    for pins in neuralOut["pins"]:
        for pin in pins:
            pinPoints.append((pin["x"], pin["y"]))

    #creates a list double in length but identical in order to lines but as individual points (x, y)
    linePoints = []
    for line in lines:
        linePoints.append((line[0], line[1]))
        linePoints.append((line[2], line[3]))

    currPoint = linePoints[1]
    unsatisfied = []
    unsatisfied.append(0)

    while linesLeft:
        net = []
        #if the direction of a line diverts by a certein threshhold of the line befor it gets counted as imtersection and is counted as Unsatisfied
        netIsFinished = FALSE
        currPoint = linesLeft[1]
        unsatisfied = []
        conected = []

        while not netIsFinished:
            
            #check for component pins if they touch and if they do thier id will be the iterator of thier list + length of linePoints
            for i in range(len(pinPoints)):
                distance = math.sqrt((currPoint(2) -pinPoints[i][0])**2 + (currPoint(3) -pinPoints[i][0])**2)
                if distance <= HITRADIUS:
                    conected.append(i + len(pinPoints))

            #check which lines "touch" each other           
            for i in range(len(linePoints)):
                distance = math.sqrt((currPoint(2) -linePoints[i][0])**2 + (currPoint(3) -linePoints[i][0])**2)
                if distance <= HITRADIUS:
                    conected.append(i)

            #if there is no point around
            if len(conected) == 0:
                if len(unsatisfied) == 0:
                    netIsFinished = TRUE
                    #break

                net.append(
                    (currPoint),
                    []
                )
            else:   
                #fill in the detected point into the net
                net.append(
                    (currPoint),
                    conected
                )
                       

            #ToDo if the line hits only the component it will fail
            #set the firstly detected points endpoint as new current Point
            if conected[0] > len(pinPoints):
                #if first conected was a pinPoint set an unsatisfied
                 if conected[0] % 2 == 0:
                    currPoint = unsatisfied[0] + 1
                    del unsatisfied[0]
                 else:
                    currPoint = unsatisfied[0] - 1
                    del unsatisfied[0] 
            elif conected[0] % 2 == 0:
                currPoint = conected[0] + 1
            else:
                currPoint = conected[0] - 1
            #add all conections which arent traced yet to the unsatisfied conections
            unsatisfied.extend(conected[1:])

        #put he dcetected net into the nets      
        nets.append(net)

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