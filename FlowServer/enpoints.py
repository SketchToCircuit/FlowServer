import json
import requests

from .src import LineDetection
from .src import Latex

#<-- Test data
NeuralTestJson = '[{"component":"R","position":{"X":130,"Y":120},"pins":[{"X":120,"Y":120},{"X":140,"Y":120}]},{"component":"C","position":{"X":160,"Y":120},"pins":[{"X":150,"Y":120},{"X":170,"Y":120}]}]'
NetTestJson = '[{"component":"R","position":{"X":130,"Y":120},"pins":[{"X":120,"Y":120,"ID":1},{"X":140,"Y":120,"ID":2}]},{"component":"C","position":{"X":160,"Y":120},"pins":[{"X":150,"Y":120,"ID":1},{"X":170,"Y":120,"ID":2}]}]'
LineTestJson = ''
#--> To Remove
class Neural:
    def __init__(self):
        self._test = ""
    async def on_post(self, req, resp):
        data = json.loads(await req.stream.read())
        outputMethodes = data["outputs"]
        imageBase64 = data["image"]

        #Send picture data to TensorflowServing for evaluation
        neuralOutput = requests.post('http://localhost:8501/v1/models/nn:predict', data = json.dumps({"image": imageBase64}))

        #Use linedetection to generate netlist and linelist
        netList,lineList = LineDetection.detect(neuralOutput, imageBase64)

        #Build output json
        output = "{"
        convert = [False,False]
        #Check which formats were chossen
        for i in range(0, len(outputMethodes)):
            if(outputMethodes[i] == "netlist"):
                convert[0] = True
            if(outputMethodes[i] == "latex"):
                convert[1] = True
        
        if(convert[0]):
            #If chossen add netlist to response
            output += "netlist:" + netList
            if(convert[1]): output += ","
        if(convert[1]):
            #If chossen add Latex code to response
            output += '"latex:"' + Latex.ListToLatex(netList,lineList)
        output += "}"

        #Send back requested data
        resp.media = json.loads(output)
        resp.status = 200