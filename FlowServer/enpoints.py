import json
from os import pardir
from tkinter.messagebox import NO
import requests
import cv2
import base64
import numpy as np

from .src import LineDetection
from .src import Latex
from .src import loging
from .src import utils

#<-- Test data
NeuralTestJson = '[{"component":"R","position":{"X":130,"Y":120},"pins":[{"X":120,"Y":120},{"X":140,"Y":120}]},{"component":"C","position":{"X":160,"Y":120},"pins":[{"X":150,"Y":120},{"X":170,"Y":120}]}]'
NetTestJson = '[{"component":"R","position":{"X":130,"Y":120},"pins":[{"X":120,"Y":120,"ID":1},{"X":140,"Y":120,"ID":2}]},{"component":"C","position":{"X":160,"Y":120},"pins":[{"X":150,"Y":120,"ID":1},{"X":170,"Y":120,"ID":2}]}]'
LineTestJson = ''
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
        form = await req.get_media()
        imageBase64 = None
        outputMethodes = None
        if(req.content_type == 'application/json'):
            try:
                outputMethodes = form["outputs"]
                imageBase64 = form["image"]
            except:
                resp.staus = 400
                return
        else:
            async for part in form:
                if part.name == 'file':
                    content = await part.data
                    nparr = np.frombuffer(content, np.uint8)
                    imageBase64 = str(base64.urlsafe_b64encode(nparr)).replace("b'", "").replace("'","")
                    outputMethodes = ["latex", "netlist"]
                else:
                    resp.staus = 400
                    return
        #Send picture data to TensorflowServing for evaluation
        image = utils.normalizeAvgLineThickness(utils.decodeBase64(imageBase64), goal_thickness=4)
        req = '{"inputs":{"image":"'+str(imageBase64)+'"}}'
        try:
            neuralOutput = requests.post('http://localhost:8501/v1/models/CompleteModel/versions/1:predict', data=req)
        except:
            loging.Warn("Serving server down")
            resp.status = 500
            return
        #Use linedetection to generate netlist and linelist
        #netList, lineList = LineDetection.detect(utils.parseNeuralOutput(neuralOutput), image)
        
        # #Build output json
        # output = "{"
        # convert = [False,False]
        # #Check which formats were chossen
        # for i in range(0, len(outputMethodes)):
        #     if(outputMethodes[i] == "netlist"):
        #         convert[0] = True
        #     if(outputMethodes[i] == "latex"):
        #         convert[1] = True
            
        # if(convert[0]):
        #     #If chossen add netlist to response
        #     output += "netlist:" + netList
        #     if(convert[1]): output += ","
        # if(convert[1]):
        #     #If chossen add Latex code to response
        #     output += '"latex:"' + Latex.ListToLatex(netList,lineList)
        # output += "}"

        # Send back requested data
        resp.media = neuralOutput.json()
        resp.status = 200
        return