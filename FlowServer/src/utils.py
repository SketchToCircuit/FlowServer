import json
import base64
import numpy as np
import cv2
import scipy.signal

#cv2.ximgproc.thinning()

_componentIDs = ['A', 'BAT', 'BTN1', 'BTN2', 'C', 'C_P', 'D', 'D_S', 'D_Z', 'F', 'GND', 'GND_C', 'GND_F', 'I1', 'I2', 'JFET_N', 'JFET_P', 'L', 'LED', 'LMP', 'M', 'MFET_N_D', 'MFET_N_E', 'MFET_P_D', 'MFET_P_E', 'MIC', 'NPN', 'OPV', 'PIN', 'PNP', 'POT', 'R', 'S1', 'S2', 'S3', 'SPK', 'U1', 'U2', 'U3', 'U_AC', 'V', 'L2']

def decodeBase64(base64img):
    decoded = base64.urlsafe_b64decode(base64img + '=' * (4 - len(base64img) % 4))
    nparr = np.fromstring(decoded, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
    return img

def normalizeAvgLineThickness(img, goal_thickness=3):
    _, bw = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    dist = cv2.distanceTransform(bw, cv2.DIST_L2, 3)
    maxima = scipy.signal.argrelextrema(dist, np.greater, order=2)
    med_thick = np.median(dist[maxima]) * 2.0
    sf = goal_thickness / med_thick
    resized = cv2.resize(img, None, fx=sf, fy=sf, interpolation=cv2.INTER_AREA)
    return cv2.threshold(resized, 0, 255, cv2.THRESH_OTSU)[1]

def parseNeuralOutput(neuralOutput):
    outputs = neuralOutput.json()['outputs']
    classes = outputs['classes']
    boxes = outputs['boxes']
    pins = outputs['pins']
    pin_cmp_ids = outputs['pin_cmp_ids']

    result = []

    for i, (cls_id, box) in enumerate(zip(classes, boxes)):
        component = {}
        # component identifier (string)
        component['component'] = _componentIDs[cls_id]
        component['topleft'] = { 'x': box[1], 'y': box[0] }
        component['bottomright'] = { 'x': box[3], 'y': box[2] }

        pin_list = [pin for pin, pin_cmp_id in zip(pins, pin_cmp_ids) if pin_cmp_id == i]
        component['pins'] = [{'x': p[0], 'y': p[1]} for p in pin_list]

        result.append(component)
    return result