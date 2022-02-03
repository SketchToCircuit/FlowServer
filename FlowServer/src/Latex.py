import json
ScalingFactor = 12.5

def ListToLatex(netlist, linelist):
    scale = getScale(netlist, linelist)
    components = drawComponents(scale, netlist)
    lines = drawLineList(scale, linelist)
    return PrepareOutput(components, lines)

def PrepareOutput(components, lines):
    output = "\\documentclass{article}\n\\usepackage{circuitikz}\n\\begin{document}\n\\begin{circuitikz}[european,line width=1pt]\n"
    output += components + lines
    output += "\n\\end{circuitikz}\n\\end{document}"
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
        compName = netlist[i]["component"]
        if(compName == "A"):
            components += drawCommon2Pin(netlist[i], scale, "rmeter, t=A")
        elif(compName == "BAT"):
            components += drawCommon2Pin(netlist[i], scale, "battery")
        elif(compName == "BTN1"):
            components += drawCommon2Pin(netlist[i], scale, "push button")
        elif(compName == "BTN2"):
            components += drawCommon2Pin(netlist[i], scale, "normally closed push button")
        elif(compName == "C"):
            components += drawCommon2Pin(netlist[i], scale, "C")
        elif(compName == "C_P"):
            components += drawCommon2Pin(netlist[i], scale, "polar capacitor")
        elif(compName == "D"):
            components += drawCommon2Pin(netlist[i], scale, "empty diode")
        elif(compName == "D_S"):
            components += drawCommon2Pin(netlist[i], scale, "empty Schottky diode")
        elif(compName == "D_Z"):
            components += drawCommon2Pin(netlist[i], scale, "full Zener diode")
        elif(compName == "F"):
            components += drawCommon2Pin(netlist[i], scale, "fuse")
        elif(compName == "GND"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)*2.5
            components += drawCommon1Pin(netlist[i], scale, "ground,scale =" + str(compscale))
        elif(compName == "GND_C"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)*2.5
            components += drawCommon1Pin(netlist[i], scale, "rground,scale =" + str(compscale))
        elif(compName == "GND_F"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)*2.5
            components += drawCommon1Pin(netlist[i], scale, "nground,scale =" + str(compscale))
        elif(compName == "I1"):
            components += drawCommon2Pin(netlist[i], scale, "european current source")
        elif(compName == "I2"):
            components += drawCommon2Pin(netlist[i], scale, "american current source")
        elif(compName == "JFET_N"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "JFET_N" + str(i), "njfet, scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "JFET_P"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "JFET_P" + str(i), "pjfet, scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "L"):
            components += drawCommon2Pin(netlist[i], scale, "inductor")
        elif(compName == "LED"):
            components += drawCommon2Pin(netlist[i], scale, "full led")
        elif(compName == "LMP"):
            components += drawCommon2Pin(netlist[i], scale, "lamp")
        elif(compName == "M"):
            components += drawCommon2Pin(netlist[i], scale, "rmeter, t=M")
        elif(compName == "MFET_N_D"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "MFET_N_D" + str(i), "nigfetd, solderdot,scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "MFET_N_E"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2#Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "MFET_N_E" + str(i), "nigfete, solderdot,scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "MFET_P_D"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "MFET_P_D" + str(i), "pigfetd,solderdot,yscale=-1,scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "MFET_P_E"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
            components += drawCommon3Pin(netlist[i], scale, "MFET_P_E" + str(i), "pigfete,solderdot,yscale=-1,scale ="+ str(compscale), "G", "D", "S")
        elif(compName == "MIC"):
            components += drawCommon2Pin(netlist[i], scale, "mic")
        elif(compName == "NPN"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Collector and Emitter for scale reference
            components += drawCommon3Pin(netlist[i], scale, "NPN" + str(i), "npn, scale ="+ str(compscale), "B", "C", "E")
        elif(compName == "OPV"):
            compscale = abs(netlist[i]["pins"][0]["y"]*scale - netlist[i]["pins"][1]["y"]*scale) #Use + and - connectors of up amp for scale reference
            components += drawCommon3Pin(netlist[i], scale, "OPV" + str(i), "op amp,yscale=-1, scale =" + str(compscale), "+","-","out")
        elif(compName == "PIN"):
            components += drawPin(netlist[i], scale, "short,o-")
        elif(compName == "PNP"):
            compscale = abs(netlist[i]["pins"][1]["y"]*scale - netlist[i]["pins"][2]["y"]*scale)/2 #Use Collector and Emitter for scale reference
            components += drawCommon3Pin(netlist[i], scale, "PNP" + str(i), "pnp, scale ="+ str(compscale), "B", "E", "C")
        elif(compName == "POT"):
            components += drawCommon3Pin(netlist[i], scale, "POT" + str(i), "genericpotentiometershape", "wiper", "left", "right")
        elif(compName == "R"):
            components += drawCommon2Pin(netlist[i], scale, "R")
        elif(compName == "S1"):
            components += drawCommon2Pin(netlist[i], scale, "cspst")
        elif(compName == "S2"):
            components += drawCommon2Pin(netlist[i], scale, "ospst")
        elif(compName == "S3"):
            components += drawCommon3Pin(netlist[i], scale, "S3"+str(i), "spdt", "in", "out 1", "out 2")
        elif(compName == "SPK"):
            components += drawCommon2Pin(netlist[i], scale, "loudspeaker")
        elif(compName == "U1"):
            components += drawCommon2Pin(netlist[i], scale, "american voltage source")
        elif(compName == "U2"):
            components += drawCommon2Pin(netlist[i], scale, "european voltage source")
        elif(compName == "U3"):
            components += drawCommon2Pin(netlist[i], scale, "battery1")
        elif(compName == "U_AC"):
            components += drawCommon2Pin(netlist[i], scale, "vsourcesin")
        elif(compName == "V"):
            components += drawCommon2Pin(netlist[i], scale, "rmeter, t=V")
        elif(compName == "L2"):
            components += drawCommon2Pin(netlist[i], scale, "american current source")

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
        lines = "\draw (" + str(point["pos"]["x"]*scale) + "," + str(ScalingFactor-point["pos"]["y"]*scale) + ") to [short] (" + str(net["points"][point["connected"][0]]["pos"]["x"]*scale) + "," + str(ScalingFactor - net["points"][point["connected"][0]]["pos"]["y"]*scale) + ");\n"            
        if(net["points"].index(point) < point["connected"][0]):
            lines += drawLine(net["points"][point["connected"][0]], net, scale)
        return lines

def drawPin(component, scale, type):
    return "\draw (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") to ["+type+"] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ");\n"

def drawCommon1Pin(component, scale, type):
    return "\draw (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") -- (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") node["+type+"]{};\n"

def drawCommon2Pin(component, scale, type):
    print("test")
    return "\draw (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") to["+type+"] (" + str(component["pins"][1]["x"]*scale) + "," + str(ScalingFactor-component["pins"][1]["y"]*scale) + ");\n"

def drawCommon3Pin(component, scale, nodeID, type, c1,c2,c3):
    return "\draw ("+ str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") node["+type+"]("+nodeID+") {} ("+nodeID+"."+ c1 +") to[short] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") ("+nodeID+"."+c2+") to[short] (" + str(component["pins"][1]["x"]*scale) + "," + str(ScalingFactor-component["pins"][1]["y"]*scale) + ") ("+nodeID+"."+c3+") to[short] (" + str(component["pins"][2]["x"]*scale) + "," + str(ScalingFactor-component["pins"][2]["y"]*scale) + ");\n"