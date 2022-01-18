import json


def ListToLatex(netlist):
    #x cord range 0 - 10
    xMax = 0
    for i in range(0, len(netlist)):
        for x in range(0, len(netlist[i]["pins"])):
            if(netlist[i]["pins"][x][0] > xMax):
                xMax = netlist[i]["pins"][x][0]

    output = "["

    for i in range(0, len(netlist)):
        if(netlist[i]["component"] == "R"):
            output += '["\draw (' + str(netlist[i]["pins"][0][0]/xMax*10) + ',' + str(netlist[i]["pins"][0][1]/xMax*10) + ') to[R,l=$R_1$, -] (' + str(netlist[i]["pins"][1][0]/xMax*10) + ',' + str(netlist[i]["pins"][1][1]/xMax*10) + ');"]'
            if(i < len(netlist)-1):
                output+=","
    output += "]"
    return output