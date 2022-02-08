from base64 import b64decode
import sys
import math
import cv2 as cv
import numpy as np
import json
from jsonc_parser.parser import JsoncParser

#Workflow detect net
#start at begining of the components
#go to first endpoint compare endpoint to nearest point and make the conection
#then go to the endpoint of the conected line and repeat as long as you end with a Circuit Copmonents pin then go to an usatisfie conection 
#whilst doing that if there are more than one hit(a junction) in the radius set them as "unsatisfied pins"
#alle lines in der nähe von von components die nicht in der richtung des components wegehen
#werden geditched (diese überprüfung nur dan wenn sie schon in der bounding + 10% boxliegen)
#the function ends if all lines are satisfied
#a line is setisfied when it got acknoledgd as useles or none of its pins/conected lines are unsatisfied

#gepfuschte static vars fürs erste
satisfied = 0

def detectNet(lines, neuralOut):

    if (satisfied == len(lines)):
        #export the nets to json
        return None
    
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
                "x": topleft[0] + (abs(topleft[0] - botright[0]) / 2),
                "y": topleft[1] + (abs(topleft[1] - botright[1]) / 2)},
            "pins": pins
            })

    return reorderPins(NetList)

def detect(img, neuralOut):
    cdstP = np.copy(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
    _, img = cv.threshold(img,128, 255, cv.THRESH_BINARY_INV)

    #kernel = np.ones((5,5),np.uint8)
    #img = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)
    _, img = cv.threshold(img,128, 255, cv.THRESH_BINARY)
    rows = img.shape[0]

    linesP = cv.HoughLinesP(img, 1, np.pi / 180, 50, 10, 10)

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
    #detect(img, jsonObject)
    
    netList = NetListExP(neuralOutput)

    netList = reorderPins(netList)

    opv = [cmp for cmp in netList if cmp['component'] == 'OPV']
    npn = [cmp for cmp in netList if cmp['component'] == 'NPN']
    print(opv)
    print(npn)

if __name__ == "__main__":
    main()