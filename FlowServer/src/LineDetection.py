import sys
import math
import cv2 as cv
import numpy as np

def detect(img):
    
    cdstP = np.copy(cv.cvtColor(img, cv.COLOR_GRAY2BGR))
    _, img = cv.threshold(img,128, 255, cv.THRESH_BINARY_INV)
    
    linesP = cv.HoughLinesP(img, 1, np.pi / 180, 50, 15, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
    
    cv.imshow("Source", img)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.waitKey()
    return 0

def main():
    print("-----")
    img = cv.imread(cv.samples.findFile("test2.jpg"), cv.IMREAD_GRAYSCALE)
    detect(img)

if __name__ == "__main__":
    main()
