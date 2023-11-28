import vrep_api.sim as sim
import sys


########################## V-REP data transimission mode ##########################
WAIT      = sim.simx_opmode_oneshot_wait  # the function will wait for the actual reply and return it
BLOCKING  = sim.simx_opmode_blocking      # it is same as WAIT
ONESHOT   = sim.simx_opmode_oneshot       # the function does not wait for the actual reply
STREAMING = sim.simx_opmode_streaming     # 
BUFFER    = sim.simx_opmode_buffer        # The cmd is not sent nor does the function wait for the actual reply

def Vrep_connect():
    # Requests a start of a simulation (connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    # print(clientID)

    # clientID stores the ID assigned by CoppeliaSim, if the connection failed it will assign -1
    if clientID!= -1:
        # print("Connected to remote server")
        pass
    else:
        print('Connection not successful')
        sim.simxFinish(-1)
        sys.exit('Could not connect')
    return clientID

def Vrep_disconnect(clientID):
    sim.simxFinish(clientID)
    # print("disconnected to remote server")

def Vrep_stop(clientID):
    sim.simxStopSimulation(clientID, WAIT)

def Vrep_start(clientID):
    sim.simxStartSimulation(clientID, WAIT)

def Vrep_pause(clientID):
    sim.simxPauseSimulation(clientID, WAIT)

def Vrep_callLuafunction(clientID, functionName, 
                    inputInts = [], inputFloats = [], inputStrings = [], inputBuffer  = bytearray()):
    
    scriptName = '/UR5'
    returnCode, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
        clientID, scriptName, 1,
        functionName, inputInts, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking
    )
    return outInts, outFloats, outStrings, outBuffer

