from asyncio.windows_events import NULL
import sys
import math
import cv2 as cv
import numpy as np
import json
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
        return NULL
    
def NetListExP(neuralOut):
    NetList = []
    
    for comp in neuralOut:
        topleft = [comp["topleft"]["x"], comp["topleft"]["y"]]
        botright = [comp["bottomright"]["x"], comp["bottomright"]["y"]]

        pins = []

        for pincoll in comp["pins"]:
            pins.append(
                {
                    "x": pincoll["x"], 
                    "y":  pincoll["y"],
                    "id": 0
                })

        NetList.append({
            "component": comp["component"],
            "position": { 
                "x": topleft[0] + (abs(topleft[0] - botright[0]) / 2),
                "y": topleft[1] + (abs(topleft[1] - botright[1]) / 2)},
            "pins": pins
            })
    out_file = open("NetList.json", "w")
    json.dump(NetList, out_file)
    out_file.close()

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
    return 0

def main():
    print("-----")

    with open("testdata.json") as jsonFile:
        jsonObject = json.load(jsonFile)
        jsonFile.close()

    img = cv.imread(cv.samples.findFile("test3.jpg"), cv.IMREAD_GRAYSCALE)
    #detect(img, jsonObject)
    NetListExP(jsonObject)

if __name__ == "__main__":
    main()