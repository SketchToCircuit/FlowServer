import json
import requests

from .src import LineDetection
from .src import Latex


class Neural:
    def __init__(self):
        self._test = ""
    async def on_post(self, req, resp):
        data = json.loads(await req.stream.read())
        outputMethodes = data["outputs"]
        imageBase64 = data["image"]
        neuralOutput = None#requests.post('http://localhost:8501/v1/models/nn:predict', data = json.dumps({"image": imageBase64}))
        netList = LineDetection.detect(neuralOutput)

        output = "{"
        convert = [False,False]
        for i in range(0, len(outputMethodes)):
            if(outputMethodes[i] == "netlist"):
                convert[0] = True
            if(outputMethodes[i] == "latex"):
                convert[1] = True
        
        if(convert[0]):
            output += "netlist:" + netList
            if(convert[1]): output += ","
        if(convert[1]):
            netList = data["testnet"]
            output += '"latex:"' + Latex.ListToLatex(netList)
        output += "}"

        resp.media = output#json.loads(output)
        resp.status = 200