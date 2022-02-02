import json
ScalingFactor = 12.5

def main():
    linelist = '[{"points":[{"pos":{"x":55,"y":252},"connected":[1]},{"pos":{"x":255,"y":250},"connected":[2]},{"pos":{"x":255,"y":190},"connected":[]}]},{"points":[{"pos":{"x":25,"y":135},"connected":[1]},{"pos":{"x":25,"y":40},"connected":[2]},{"pos":{"x":255,"y":40},"connected":[3]},{"pos":{"x":255,"y":100},"connected":[]}]},{"points":[{"pos":{"x":145,"y":145},"connected":[1,2]},{"pos":{"x":205,"y":145},"connected":[]},{"pos":{"x":145,"y":190},"connected":[3]},{"pos":{"x":25,"y":190},"connected":[4]},{"pos":{"x":25,"y":155},"connected":[]}]}]'
    netlist = '[{"component":"PIN","position":{"x":40,"y":252},"pins":[{"x":55,"y":1400,"id":0}]},{"component":"OPV","position":{"x":90,"y":145},"pins":[{"x":25,"y":135,"id":1},{"x":25,"y":155,"id":2},{"x":145,"y":145,"id":2}]},{"component":"NPN","position":{"x":255,"y":145},"pins":[{"x":205,"y":145,"id":2},{"x":255,"y":100,"id":1},{"x":255,"y":190,"id":0}]},{"component":"R","position":{"x":230,"y":200},"pins":[{"x":205,"y":145,"id":2},{"x":255,"y":100,"id":1},{"x":255,"y":190,"id":0}]}]'
    linelist = json.loads(linelist)
    netlist = json.loads(netlist)
    print(ListToLatex(netlist, linelist))

def ListToLatex(netlist, linelist):
    scale = getScale(netlist, linelist)
    components = drawComponents(scale, netlist)
    lines = drawLineList(scale, linelist)
    return PrepareOutput(components, lines)

def PrepareOutput(components, lines):
    output = "\\documentclass{article}\n\\usepackage{circuitikz}\n\\begin{document}\n\\begin{circuitikz}[european,line width=1pt]\n"
    output += components + lines
    output += "\n\\end{circuitikz}\n\\end{document}"
    output = '"latex":"' + output + '"'
    return output

def getScale(netlist, linelist):
    #x cord range 0 - ScalingFactor bigger range possible -5 to +ScalingFactor
    xMax = 0
    yMax = 0
    for i in range(0,len(netlist)):
        for x in range(0, len(netlist[i]["pins"])):
            if(yMax < netlist[i]["pins"][x]["y"]):
                yMax = netlist[i]["pins"][x]["y"]
            if(xMax < netlist[i]["pins"][x]["x"]):
                xMax = netlist[i]["pins"][x]["x"]
    for i in range(0,len(linelist)):
        for x in range(0, len(linelist[i]["points"])):
            for y in range(0, len(linelist[i]["points"][x])):
                if(yMax < linelist[i]["points"][y]["pos"]["y"]):
                    yMax = linelist[i]["points"][y]["pos"]["y"]
                if(xMax < linelist[i]["points"][y]["pos"]["x"]):
                    xMax = linelist[i]["points"][y]["pos"]["x"]
    if(yMax < xMax):
        return ScalingFactor/xMax
    else:
        return ScalingFactor/yMax

def drawComponents(scale, netlist):
    components = ""
    for i in range(0, len(netlist)):
        if(netlist[i]["component"] == "PIN"):
            components += drawPin(netlist[i], scale)
        if(netlist[i]["component"] == "OPV"):
            compscale = abs(netlist[i]["pins"][0]["y"]*scale - netlist[i]["pins"][1]["y"]*scale) #Use + and - connectors of up amp for scale reference
            components += drawCommon3Pin(netlist[i], scale, "OPV" + str(i), "op amp,yscale=-1, scale =" + str(compscale), "+","-","out")
        if(netlist[i]["component"] == "NPN"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Collector and Emitter for scale reference
            components += drawCommon3Pin(netlist[i], scale, "NPN" + str(i), "npn, scale ="+ str(compscale), "B", "C", "E")
        # if(netlist[i]["component"] == "R"):
        #     compscale = abs(netlist[i]["pins"][0]["y"]*scale - netlist[i]["pins"][1]["y"]*scale) + abs(netlist[i]["pins"][0]["x"]*scale - netlist[i]["pins"][1]["x"]*scale)
        #     components += drawCommon2Pin(netlist[i], scale, "R" + str(i), "R, scale ="+ str(compscale), "B", "C", "E")
    return components

def drawLineList(scale, linelist):
    lines = ""
    for i in range(0, len(linelist)):
        lines += drawLine(linelist[i]["points"][0], linelist[i], scale)
    return lines

def drawLine(point,net,scale):
    if(len(point["connected"])<1): return ""
    if(len(point["connected"])>1):
        lines = ""
        for i in range(0, len(point["connected"])):
            lines += "\draw (" + str(point["pos"]["x"]*scale) + "," + str(ScalingFactor-point["pos"]["y"]*scale) + ") to [short, *-] (" + str(net["points"][point["connected"][i]]["pos"]["x"]*scale) + "," + str(ScalingFactor - net["points"][point["connected"][i]]["pos"]["y"]*scale) + ");\n"
            if(net["points"].index(point) < point["connected"][i]):
                lines += drawLine(net["points"][point["connected"][i]], net, scale)
        return lines
    else:
        lines = "\draw (" + str(point["pos"]["x"]*scale) + "," + str(ScalingFactor-point["pos"]["y"]*scale) + ") to [short, --] (" + str(net["points"][point["connected"][0]]["pos"]["x"]*scale) + "," + str(ScalingFactor - net["points"][point["connected"][0]]["pos"]["y"]*scale) + ");\n"            
        if(net["points"].index(point) < point["connected"][0]):
            lines += drawLine(net["points"][point["connected"][0]], net, scale)
        return lines

def drawCommon2Pin(component, scale, type):
    return "\draw (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") to [short, o-] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ");\n"

def drawCommon3Pin(component, scale, nodeID, type, c1,c2,c3):
    return "\draw ("+ str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") node["+type+"]("+nodeID+") {} ("+nodeID+"."+ c1 +") to[short,--] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") ("+nodeID+"."+c2+") to[short,--] (" + str(component["pins"][1]["x"]*scale) + "," + str(ScalingFactor-component["pins"][1]["y"]*scale) + ") ("+nodeID+"."+c3+") to[short,--] (" + str(component["pins"][2]["x"]*scale) + "," + str(ScalingFactor-component["pins"][2]["y"]*scale) + ");\n"

def drawPin(component, scale):
    return "\draw (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") to [short, o-] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ");\n"
main()