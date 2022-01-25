from email import base64mime
import json
import requests

base64 = open('test.txt', 'r').read().replace('\n', '')
req = '{"inputs":{"image":"'+base64+'"}}'
neuralOutput = requests.post('http://localhost:8501/v1/models/CompleteModel/versions/1:predict', data=req)

data = neuralOutput.json()

with open('data.json', 'w') as f:
    json.dump(data, f)
