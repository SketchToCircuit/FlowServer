import os
from datetime import datetime

#Config
LogDir = "./FlowServer/log"
curLog = "latest.log"
#End Config

def Init():
    if(os.path.exists(os.path.join(LogDir, curLog))):
        with open(os.path.join(LogDir, curLog)) as file:
            oldLog = file.read()
            if(len(oldLog) > 1):
                with open(os.path.join(LogDir, datetime.now().strftime("%m\%d\%Y\%H:%M:%S") + ".log"), "w") as newLog:
                    newLog.write(oldLog)
            os.remove(os.path.join(LogDir, curLog))
    with open(os.path.join(LogDir, curLog), "w") as file:
        file.write("")

def Warn(msg):
    write("[Warn]" + msg)
    return

def Info(msg):
    write("[Info]" + msg)

def Exception(msg):
    write("[Error]" + msg)
    return

def write(msg):
    with open(os.path.join(LogDir, curLog), "a") as file:
        file.write("[" + datetime.now().strftime("%d/%m/%Y/%H:%M:%S") + "]" + msg + "\n")