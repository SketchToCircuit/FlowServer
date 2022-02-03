import requests
import cv2
import base64
import numpy as np
import json

from .src import LineDetection
from .src import Latex
from .src import loging
from .src import utils

#<-- Test data
Testlinelist = '[]'
Testnetlist = '[{"component": "PIN", "position": {"x": 730.0, "y": 117.5}, "pins": [{"x": 786, "y": 109, "id": 0}]}, {"component": "PIN", "position": {"x": 470.0, "y": 297.0}, "pins": [{"x": 408, "y": 289, "id": 0}]}, {"component": "PIN", "position": {"x": 868.5, "y": 299.5}, "pins": [{"x": 819, "y": 293, "id": 0}]}, {"component": "PIN", "position": {"x": 718.5, "y": 327.5}, "pins": [{"x": 768, "y": 327, "id": 0}]}, {"component": "PIN", "position": {"x": 455.5, "y": 194.5}, "pins": [{"x": 392, "y": 186, "id": 0}]}, {"component": "PIN", "position": {"x": 858.5, "y": 116.0}, "pins": [{"x": 811, "y": 116, "id": 0}]}, {"component": "GND", "position": {"x": 182.5, "y": 321.5}, "pins": [{"x": 182, "y": 272, "id": 0}]}, {"component": "C", "position": {"x": 133.5, "y": 97.5}, "pins": [{"x": 188, "y": 81, "id": 0}, {"x": 72, "y": 97, "id": 0}]}, {"component": "R", "position": {"x": 293.5, "y": 59.5}, "pins": [{"x": 220, "y": 50, "id": 0}, {"x": 359, "y": 55, "id": 0}]}, {"component": "OPV", "position": {"x": 288.5, "y": 164.0}, "pins": [{"x": 202, "y": 135, "id": 0}, {"x": 366, "y": 187, "id": 0}, {"x": 202, "y": 193, "id": 0}]}]'
#--> To Remove
class Neural:
    def __init__(self):
        loging.Init()
        loging.Info("Server up")

    async def on_get(self, req, resp):
        resp.status = 200
        resp.content_type = 'text/html'
        with open('./FlowServer/index.html', 'r') as f:
            resp.body = f.read()

    async def on_post(self, req, resp):
        loging.Info("Connection from: " + req.remote_addr)
        onWebsite = True
        try:
            form = await req.get_media()
            imageBase64 = None
            outputMethodes = []
            if(req.content_type == 'application/json'):
                onWebsite = False
                outputMethodes = form["outputs"]
                imageBase64 = form["image"]
                resp.staus = 400
            else:
                async for part in form:
                    if part.name == 'file':
                        content = await part.data
                        nparr = np.frombuffer(content, np.uint8)
                        imageBase64 = str(base64.urlsafe_b64encode(nparr)).replace("b'", "").replace("'","")
                    elif part.name == 'netList':
                        outputMethodes.append("netList")
                    elif part.name == 'lineList':
                        outputMethodes.append("lineList")
                    elif part.name == 'latex':
                        outputMethodes.append("latex")
        except:
            resp.staus = 400
            return
        # if(imageBase64 == "" or len(outputMethodes)<1):
        #     resp.status = 400
        #     return
        # #Send picture data to TensorflowServing for evaluation
        # image = utils.normalizeAvgLineThickness(utils.decodeBase64(imageBase64), goal_thickness=4)
        # nnreq = '{"inputs":{"image":"'+str(base64.urlsafe_b64encode(cv2.imencode('.jpg', image)[1])).replace("b'", "").replace("'","")+'"}}'
        # try:
        #     neuralOutput = requests.post('http://localhost:8501/v1/models/CompleteModel/versions/1:predict', data=nnreq)
        # except:
        #     loging.Warn("Serving server down")
        #     resp.status = 500
        #     return
        # #Use linedetection to generate netlist and linelist
        # #netList, lineList = LineDetection.detect(utils.parseNeuralOutput(neuralOutput), image)#

        netList = json.loads(Testnetlist)
        lineList = json.loads(Testlinelist)
        output = {
        }
        if("netList" in outputMethodes): output["netList"] = netList
        if("lineList" in outputMethodes): output["lineList"] = lineList
        if("latex" in outputMethodes): output["latex"] = Latex.ListToLatex(netList, lineList)
        # Send back requested data
        if(onWebsite):
            msg = ""
            if("netList" in output): msg += "\n"+json.dumps(output["netList"])
            if("lineList" in output): msg += "\n"+json.dumps(output["lineList"])
            if("latex" in output): msg += "\n"+output["latex"]
            resp.text = msg
        else:
            resp.media = output
            resp.status = 200
            return