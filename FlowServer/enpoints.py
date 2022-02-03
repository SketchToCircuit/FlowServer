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
Testlinelist = '[{"points":[{"pos":{"x":55,"y":252},"connected":[1]},{"pos":{"x":255,"y":250},"connected":[2]},{"pos":{"x":255,"y":190},"connected":[]}]},{"points":[{"pos":{"x":25,"y":135},"connected":[1]},{"pos":{"x":25,"y":40},"connected":[2]},{"pos":{"x":255,"y":40},"connected":[3]},{"pos":{"x":255,"y":100},"connected":[]}]},{"points":[{"pos":{"x":145,"y":145},"connected":[1,2]},{"pos":{"x":205,"y":145},"connected":[]},{"pos":{"x":145,"y":190},"connected":[3]},{"pos":{"x":25,"y":190},"connected":[4]},{"pos":{"x":25,"y":155},"connected":[]}]}]'
Testnetlist = '[{"component":"PIN","position":{"x":40,"y":252},"pins":[{"x":55,"y":255,"id":0}]},{"component":"L2","position":{"x":100,"y":100},"pins":[{"x":50,"y":50,"id":99},{"x":150,"y":150,"id":99}]},{"component":"OPV","position":{"x":90,"y":145},"pins":[{"x":25,"y":135,"id":1},{"x":25,"y":155,"id":2},{"x":145,"y":145,"id":2}]},{"component":"S3","position":{"x":255,"y":145},"pins":[{"x":205,"y":145,"id":2},{"x":255,"y":100,"id":1},{"x":255,"y":190,"id":0}]},{"component":"R","position":{"x":230,"y":200},"pins":[{"x":205,"y":145,"id":2},{"x":255,"y":100,"id":1},{"x":255,"y":190,"id":0}]}]'
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