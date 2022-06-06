import requests
import cv2
import base64
import numpy as np
import json
import traceback

from .src import LineDetection
from .src import Latex
from .src import loging
from .src import utils

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
            loging.Warn("Exception: " + traceback.format_exc())
            resp.staus = 400
            return
        if(imageBase64 == "" or len(outputMethodes)<1):
            resp.status = 400
            return
        # Send picture data to TensorflowServing for evaluation
        image = utils.normalizeAvgLineThickness(utils.decodeBase64(imageBase64), goal_thickness=3)
        nnreq = '{"inputs":{"image":"'+str(base64.urlsafe_b64encode(cv2.imencode('.jpg', image)[1])).replace("b'", "").replace("'","")+'"}}'
        try:
            neuralOutput = requests.post('http://localhost:8501/v1/models/CompleteModel/versions/1:predict', data=nnreq)
        except:
            loging.Warn("Serving server down")
            resp.status = 400
            return


        if(len(neuralOutput.json()["outputs"]["pins"]) < 1):
            loging.Warn("Empty output")
            resp.status = 500
            return

        try:
            # Use linedetection to generate netlist and linelist
            netList, lineList = LineDetection.detect(utils.parseNeuralOutput(neuralOutput), image)

            output = {
            }

            if("netList" in outputMethodes): output["netList"] = netList
            if("lineList" in outputMethodes): output["lineList"] = lineList
            if("latex" in outputMethodes): output["latex"] = Latex.ListToLatex(netList, lineList)
        except Exception:
            loging.Warn("Exception: " + traceback.format_exc())
            resp.staus = 400
            return

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