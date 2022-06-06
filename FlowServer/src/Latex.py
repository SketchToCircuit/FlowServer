import math

ScalingFactor = 11.5

def ListToLatex(netlist, linelist):
    scale = getScale(netlist, linelist)
    components = drawComponents(scale, netlist)
    lines = drawLineList(scale, linelist, netlist)
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
        if(yMax < netlist[i]["position"]["y"]):
            yMax = netlist[i]["position"]["y"]
        if(xMax < netlist[i]["position"]["x"]):
            xMax = netlist[i]["position"]["x"]
        for x in range(0, len(netlist[i]["pins"])):
            if(yMax < netlist[i]["pins"][x]["y"]):
                yMax = netlist[i]["pins"][x]["y"]
            if(xMax < netlist[i]["pins"][x]["x"]):
                xMax = netlist[i]["pins"][x]["x"]
    for i in range(0,len(linelist)):
        for x in range(0, len(linelist[i]["points"])):
            if(yMax < linelist[i]["points"][x]["pos"]["y"]):
                yMax = linelist[i]["points"][x]["pos"]["y"]
            if(xMax < linelist[i]["points"][x]["pos"]["x"]):
                xMax = linelist[i]["points"][x]["pos"]["x"]
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
            components += drawCommon2Pin(netlist[i], scale, "curved capacitor")
        elif(compName == "D"):
            components += drawCommon2Pin(netlist[i], scale, "empty diode")
        elif(compName == "D_S"):
            components += drawCommon2Pin(netlist[i], scale, "empty Schottky diode")
        elif(compName == "D_Z"):
            components += drawCommon2Pin(netlist[i], scale, "full Zener diode")
        elif(compName == "F"):
            components += drawCommon2Pin(netlist[i], scale, "fuse")
        elif(compName == "GND"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)
            components += drawCommon1Pin(netlist[i], scale, "ground,scale =" + str(compscale))
        elif(compName == "GND_C"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)
            components += drawCommon1Pin(netlist[i], scale, "rground,scale =" + str(compscale))
        elif(compName == "GND_F"):
            compscale = abs(netlist[i]["position"]["y"]*scale - netlist[i]["pins"][0]["y"]*scale)
            components += drawCommon1Pin(netlist[i], scale, "nground,scale =" + str(compscale))
        elif(compName == "I1"):
            components += drawCommon2Pin(netlist[i], scale, "european current source")
        elif(compName == "I2"):
            components += drawCommon2Pin(netlist[i], scale, "american current source")
        elif(compName == "L"):
            components += drawCommon2Pin(netlist[i], scale, "inductor")
        elif(compName == "LED"):
            components += drawCommon2Pin(netlist[i], scale, "full led")
        elif(compName == "LMP"):
            components += drawCommon2Pin(netlist[i], scale, "lamp")
        elif(compName == "M"):
            components += drawCommon2Pin(netlist[i], scale, "rmeter, t=M")
        elif(compName == "MIC"):
            components += drawCommon2Pin(netlist[i], scale, "mic")
        elif(compName == "OPV"):
            compscale = abs(netlist[i]["pins"][0]["y"]*scale - netlist[i]["pins"][1]["y"]*scale)/2 #Use + and - connectors of up amp for scale reference
            if(netlist[i]["pins"][0]["y"] > netlist[i]["pins"][1]["y"]):
                components += drawCommon3Pin(netlist[i], scale, "OPV" + str(i), "op amp,yscale=-1,scale =" + str(compscale), "-","+","out")
            else:
                components += drawCommon3Pin(netlist[i], scale, "OPV" + str(i), "op amp, scale =" + str(compscale), "-","+","out")
        elif(compName == "PIN"):
            components += drawPin(netlist[i], scale, "short,o-")
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
        #Transistor
        elif(compName == "JFET_N"):
            components += drawTransistor(i,netlist[i], scale,"njfet", True)
        elif(compName == "JFET_P"):
            components += drawTransistor(i,netlist[i], scale,"pjfet", True)
        elif(compName == "MFET_N_D"):
            components += drawTransistor(i,netlist[i], scale,"nigfetd, solderdot", True)
        elif(compName == "MFET_N_E"):
            components += drawTransistor(i,netlist[i], scale,"nigfete, solderdot", True)
        elif(compName == "MFET_P_D"):
            components += drawTransistor(i,netlist[i], scale,"pigfetd,solderdot,yscale=-1", True)
        elif(compName == "MFET_P_E"):
            components += drawTransistor(i,netlist[i], scale,"pigfete,solderdot,yscale=-1", True)
        elif(compName == "NPN"):
            components += drawTransistor(i,netlist[i], scale, "npn", False)
        elif(compName == "PNP"):
            components += drawTransistor(i,netlist[i], scale, "pnp", False)
    return components

def drawTransistor(id,component, scale ,type, fet):
    component["position"]["y"] = component["pins"][0]["y"]
    component["position"]["x"] = component["pins"][1]["x"]
    compscale = abs(component["pins"][1]["y"]*scale - component["pins"][2]["y"]*scale)/2 #Use Drain and Source for scale reference
    if(fet):
        c1 = "G"
        c2 = "D"
        c3 = "S"
    else:
        c1 = "B"
        c2 = "C"
        c3 = "E"
    if(10-component["pins"][1]["y"]*scale < 10-component["pins"][2]["y"]*scale):
        tmp = c2
        c2 = c3
        c3 = tmp
    return drawCommon3Pin(component, scale, "Transistor" + str(id), type + ", scale ="+ str(compscale),c1,c2,c3)

def drawPin(component, scale, type):
    return "\draw (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") to ["+type+"] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ");\n"

def drawCommon1Pin(component, scale, type):
    return "\draw (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") -- (" + str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") node["+type+"]{};\n"

def drawCommon2Pin(component, scale, type):
    return "\draw (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") to["+type+"] (" + str(component["pins"][1]["x"]*scale) + "," + str(ScalingFactor-component["pins"][1]["y"]*scale) + ");\n"

def drawCommon3Pin(component, scale, nodeID, type, c1,c2,c3):
    return "\draw ("+ str(component["position"]["x"]*scale) + "," + str(ScalingFactor-component["position"]["y"]*scale) + ") node["+type+"]("+nodeID+") {} ("+nodeID+"."+ c1 +") to[short] (" + str(component["pins"][0]["x"]*scale) + "," + str(ScalingFactor-component["pins"][0]["y"]*scale) + ") ("+nodeID+"."+c2+") to[short] (" + str(component["pins"][1]["x"]*scale) + "," + str(ScalingFactor-component["pins"][1]["y"]*scale) + ") ("+nodeID+"."+c3+") to[short] (" + str(component["pins"][2]["x"]*scale) + "," + str(ScalingFactor-component["pins"][2]["y"]*scale) + ");\n"

def drawLineList(scale, linelist, netList):
    lines = ""
    for i in range(0, len(linelist)):
        lines += drawLine(linelist[i]["points"][0], linelist[i], scale, netList)
    return lines

def numConnections(pt, net, netList):
    num = len(pt["connected"])
    for n in netList:
        for pin in n["pins"]:
            dist = (pin["x"] - pt["pos"]["x"])**2 + (pin["y"] - pt["pos"]["y"])**2
            if dist < 1:
                num += 1
    curr_idx = 0
    for i, p in enumerate(net['points']):
        if p == pt:
            curr_idx = i
            break
    for p in net['points']:
        if curr_idx in p['connected']:
            num += 1
    return num

def drawLine(point,net,scale, netList):
    if(len(point["connected"])<1): return ""
    lines = ""
    for i in range(0, len(point["connected"])):
        if numConnections(point, net, netList) > 2:
            thick_point_str = ", *-"
        else:
            thick_point_str = ", -"
        
        if numConnections(net["points"][point["connected"][i]], net, netList) > 2:
            thick_point_str += "*"
        
        lines += "\draw (" + str(point["pos"]["x"]*scale) + "," + str(ScalingFactor-point["pos"]["y"]*scale) + ") to [short"+thick_point_str+"] (" + str(net["points"][point["connected"][i]]["pos"]["x"]*scale) + "," + str(ScalingFactor - net["points"][point["connected"][i]]["pos"]["y"]*scale) + ");\n"
        if(net["points"].index(point) < point["connected"][i]):
            lines += drawLine(net["points"][point["connected"][i]], net, scale, netList)
    return lines
    # else:
    #     thick_point_str = ""


    #     lines = "\draw (" + str(point["pos"]["x"]*scale) + "," + str(ScalingFactor-point["pos"]["y"]*scale) + ") to [short"+thick_point_str+"] (" + str(net["points"][point["connected"][0]]["pos"]["x"]*scale) + "," + str(ScalingFactor - net["points"][point["connected"][0]]["pos"]["y"]*scale) + ");\n"            
    #     if(net["points"].index(point) < point["connected"][0]):
    #         lines += drawLine(net["points"][point["connected"][0]], net, scale)
    #     return lines