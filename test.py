import json

netlist = json.loads('[{"component":"R","position":[120,120],"pins":[[120,120,1],[120,120,1]]},{"component":"C","position":[240,240],"pins":[[240,240,1],[240,240,1]]}]')
yMax = 0
xMax = 0
for i in range(0, len(netlist)):
    for x in range(0, len(netlist[i]["pins"])):
        if(netlist[i]["pins"][x][0] > xMax):
            xMax = netlist[i]["pins"][x][0]
        if(netlist[i]["pins"][x][1] > yMax):
            yMax = netlist[i]["pins"][x][1]

print(yMax, xMax)