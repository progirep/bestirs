#!/usr/bin/python
#
# Policy Visualizer

import math
import os
import sys, code, random
import resource
import subprocess
import signal, time
import tempfile
import threading
import itertools
import Queue
import traceback
from PIL import Image
import os, pygame, pygame.locals

# ==================================
# Settings
# ---> Need to be adapted for different settings,
#      Goals, and dynamics
# ==================================
CONFIGURATION_FILE_START = """nofDimensions=4
nofTranslationInvariantDimensions=2
SolverMode=GeneralizedBuchi
"""
# GOALS = [((5,10),(2,9)),((5,10),(20,25))]
# BADREGIONS = [((30,35),(15,25)),((40,45),(20,25)),((0,40),(10,15))]
# XSIZE = 64
# YSIZE = 32
GOALS = [ ((0,5),(0,10)),  ((5,10),(20,30)) , ((25,31),(0,10))  ] # [((0,9),(9,15)),((20,31),(0,8)) ]
BADREGIONS = [((10,14),(10,31)), ((21,24),(0,17))] # [((10,12),(0,6))]
XSIZE = 32
YSIZE = 32
STARTING_STATE = (0,0,0,0)

# Simulator options
SIMULATION_STEPS_PER_TIME_STEP = 30
ODE_STEPS_PER_SIMULATION_STEP = 100
FPS = 60
LENGTH_TRAIL = 300


# Continuous DYNAMICS
STATE_ETA = (0.2,0.2,0.2,0.2)
STATE_LB = (-0.5*STATE_ETA[0],-0.5*STATE_ETA[1],-1.0-0.5*STATE_ETA[2],-1.0-0.5*STATE_ETA[3])
STATE_UB = ((XSIZE+0.5)*STATE_ETA[0],(YSIZE+0.5)*STATE_ETA[1],1.1,2.1)
INPUT_LB = (-0.7,-1.5)
INPUT_UB = (0.7,0.1)
INPUT_ETA = (0.2,0.2)
TIME_STEP = 0.8
def getGradient(x,u):
    xx = [0,0,0,0]
    xx[0] = x[2]
    xx[1] = x[3]
    xx[2] = u[0]
    xx[3] = 1.0+u[1]
    return tuple(xx)


# ==================================================
# Derivated values
# ==================================================
INPUT_NOF_VALUES_PER_DIMENSION = [ int(INPUT_UB[i]/INPUT_ETA[i])-int(INPUT_LB[i]/INPUT_ETA[i])+1 for i in range(0,len(INPUT_LB))]
INPUT_MIN_VALUE = [ int(INPUT_LB[i]/INPUT_ETA[i])*INPUT_ETA[i] for i in range(0,len(INPUT_LB))]
STATE_MIN_VALUE = [ int(STATE_LB[i]/STATE_ETA[i])*STATE_ETA[i] for i in range(0,len(STATE_LB))]


CONFIGURATION_FILE_PATH = "/tmp/setting"+str(os.getpid())+".txt"
TRANSITION_FILE_PATH = "/tmp/transitions"+str(os.getpid())+".txt"

# ==================================
# Complete configuration file
# ==================================
configurationFile = CONFIGURATION_FILE_START+"\n"
configurationFile = configurationFile+"\nWorkspaceSizeDim0="+str(XSIZE)
configurationFile = configurationFile+"\nWorkspaceSizeDim1="+str(YSIZE)
configurationFile = configurationFile+"\nNofBadRegions="+str(len(BADREGIONS))
configurationFile = configurationFile+"\nNofGoals="+str(len(GOALS))
for i in range(0,len(GOALS)):
    configurationFile = configurationFile+"\nGoal"+str(i)+"="+str(GOALS[i][0][0])+"-"+str(GOALS[i][0][1])+","+str(GOALS[i][1][0])+"-"+str(GOALS[i][1][1])+",*,*"
for i in range(0,len(BADREGIONS)):
    configurationFile = configurationFile+"\nBadRegion"+str(i)+"="+str(BADREGIONS[i][0][0])+"-"+str(BADREGIONS[i][0][1])+","+str(BADREGIONS[i][1][0])+"-"+str(BADREGIONS[i][1][1])+",*,*"
configurationFile = configurationFile+"\n"
with open(CONFIGURATION_FILE_PATH,"w") as outFile:
    outFile.write(configurationFile)

# ==================================
# Run the Abstraction generator plugin
# ==================================
errorLevel = os.system("./generator "+TRANSITION_FILE_PATH)
if errorLevel!=0:
    print >>sys.stderr,"Error running the generator!\n"
    sys.exit(1)


# ==================================
# Adapt size of display
# ==================================
pygame.init()
displayInfo = pygame.display.Info()
MAGNIFY = displayInfo.current_w
MAGNIFY = min(MAGNIFY,(displayInfo.current_w-50)/(XSIZE+2))
MAGNIFY = min(MAGNIFY,(displayInfo.current_h-50)/(YSIZE+2))


# ==================================
# Background computation loop
# ==================================
dataQueue = Queue.Queue(100000)

class RunnerThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.stopped = False
        self.reset = True

    def run(self):
        try:
            currentState = STARTING_STATE
            nextStates = []

            # Open The program
            runnerProcess = subprocess.Popen("../Solver/solver --interactive --multiStepMultiAction "+TRANSITION_FILE_PATH+" "+CONFIGURATION_FILE_PATH, shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            self.runnerProcess = runnerProcess

            while True:
                if (self.stopped):
                    return

                if self.reset:
                    currentState = tuple([random.randint(0,XSIZE-1),random.randint(0,YSIZE-1)]+list(currentState)[2:])
                    CONTINUOUS_STATE = [a*STATE_ETA[i]+STATE_MIN_VALUE[i] for (i,a) in enumerate(currentState)]
                    nextStates = []
                    self.reset = False
                    dataQueue.put(None)

                # Choose next state
                if len(nextStates)>0:
                    currentState = tuple(random.choice(nextStates))

                # First state or initial case
                print "Writing: ",currentState
                sys.stdout.flush()
                for a in currentState:
                    runnerProcess.stdin.write(str(a)+"\n")
                runnerProcess.stdin.flush()
                print "Reading"
                sys.stdout.flush()
                goalLine = runnerProcess.stdout.readline()
                print goalLine
                sys.stdout.flush()
                while (goalLine=="LOSINGSTATE\n"):
                    time.sleep(15)
                    # Randomize X/Y
                    currentState = tuple([random.randint(0,XSIZE-1),random.randint(0,YSIZE-1)]+list(currentState)[2:])
                    for a in currentState:
                        runnerProcess.stdin.write(str(a)+"\n")
                    runnerProcess.stdin.flush()
                    goalLine = runnerProcess.stdout.readline()
                    print "LAST:"+str(currentState)+"now: "+goalLine
                    sys.stdout.flush()
                    CONTINUOUS_STATE = [a*STATE_ETA[i]+int(STATE_MIN_VALUE[i]/STATE_ETA[i])*STATE_ETA[i] for (i,a) in enumerate(currentState)]
                    # time.sleep(1)
                assert goalLine[0]=="C"
                actionLine = runnerProcess.stdout.readline()
                print actionLine
                sys.stdout.flush()
                assert actionLine.startswith("Action ")
                successorLine = runnerProcess.stdout.readline()
                print successorLine
                sys.stdout.flush()
                nextStates = []
                while (successorLine)!="DONE\n":
                    succ = [int(a) for a in successorLine.strip().split(",")]
                    nextStates.append(succ)
                    successorLine = runnerProcess.stdout.readline()
                    print successorLine
                    sys.stdout.flush()

                # Simulate
                continuousInput = [0 for i in range(0,len(INPUT_LB))]
                actionNum = int(actionLine.strip().split(" ")[1])
                for i in range(0,len(INPUT_LB)):
                    localActionNum = actionNum % INPUT_NOF_VALUES_PER_DIMENSION[i]
                    actionNum = int(actionNum / INPUT_NOF_VALUES_PER_DIMENSION[i])
                    continuousInput[i] = int(INPUT_MIN_VALUE[i]/INPUT_ETA[i])*INPUT_ETA[i]+localActionNum*INPUT_ETA[i]
                for i in range(0,SIMULATION_STEPS_PER_TIME_STEP):
                    epsilon = float(TIME_STEP)/SIMULATION_STEPS_PER_TIME_STEP/ODE_STEPS_PER_SIMULATION_STEP
                    print "Continuous INput:",continuousInput
                    for j in range(0,ODE_STEPS_PER_SIMULATION_STEP):
                        xx = getGradient(CONTINUOUS_STATE,continuousInput)
                        CONTINUOUS_STATE = [xx[i]*epsilon+a for (i,a) in enumerate(CONTINUOUS_STATE)]
                    dataQueue.put((currentState,nextStates,CONTINUOUS_STATE))
                    print "Cont.State:",CONTINUOUS_STATE
                # Translate continuous state back to discrete one
                nextStates = [ [ int(CONTINUOUS_STATE[i]/STATE_ETA[i]) - int(STATE_MIN_VALUE[i]/STATE_ETA[i]) for i in range(0,len(STATE_LB)) ] ]
                print "nextStatesB: ",nextStates
        except Exception as e: 
            traceback.print_exc()
            os.kill(os.getpid(), signal.SIGKILL)
                            

# Start thread
t = RunnerThread()
t.daemon = True
t.start()

# ==================================
# Main loop
# ==================================
def actionLoop():
    screen = pygame.display.set_mode(((XSIZE+2)*MAGNIFY,(YSIZE+2)*MAGNIFY))
    clock = pygame.time.Clock()

    screenBuffer = pygame.Surface(screen.get_size())
    screenBuffer = screenBuffer.convert()

    isPaused = False
    positionHistory = []
    pygame.display.set_caption('Strategy Visualizer')
    
    # Get the initial "None" value due to the reset
    assert dataQueue.get() is None
    
    while 1:

        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT or (event.type == pygame.locals.KEYDOWN and event.key == pygame.locals.K_ESCAPE):
                t.stopped = True
                # Hard kill!
                os.kill(os.getpid(),9)
            if (event.type == pygame.locals.KEYDOWN and event.key == pygame.locals.K_r):
                t.reset = True
                while not dataQueue.get() is None:
                    pass
                positionHistory = []
            if (event.type == pygame.locals.KEYDOWN and event.key == pygame.locals.K_SPACE):
                isPaused = not isPaused
                if isPaused:
                    pygame.display.set_caption('Strategy Visualizer (Paused)')
                else:
                    pygame.display.set_caption('Strategy Visualizer')

        # Draw Field
        screenBuffer.fill((64, 64, 64)) # Dark Gray
        screenBuffer.fill((128, 128, 128),pygame.Rect(1*MAGNIFY,1*MAGNIFY,XSIZE*MAGNIFY,YSIZE*MAGNIFY)) # Lighter Dray

        for ((a,b),(c,d)) in GOALS:
            screenBuffer.fill((0,128,0),pygame.Rect((a+1)*MAGNIFY,(c+1)*MAGNIFY,(b+1-a)*MAGNIFY,(d+1-c)*MAGNIFY))
        for ((a,b),(c,d)) in BADREGIONS:
            screenBuffer.fill((0,0,0),pygame.Rect((a+1)*MAGNIFY,(c+1)*MAGNIFY,(b+1-a)*MAGNIFY,(d+1-c)*MAGNIFY))

        # Updated currentState and nextStates
        if (not isPaused) or len(nextStates)==0:
            (currentState,nextStates,continuousState) = dataQueue.get()
            positionHistory.append(continuousState)
            if len(positionHistory)>LENGTH_TRAIL:
                positionHistory = positionHistory[1:]

        # Draw next states
        for nextState in nextStates:
            screenBuffer.fill((0,0,128),pygame.Rect((nextState[0]+1)*MAGNIFY,(nextState[1]+1)*MAGNIFY,MAGNIFY,MAGNIFY))

        # Draw current state
        screenBuffer.fill((128,0,0),pygame.Rect((currentState[0]+1)*MAGNIFY,(currentState[1]+1)*MAGNIFY,MAGNIFY,MAGNIFY))

        # Draw position history
        for i in range(0,len(positionHistory)-1):
            pygame.draw.line(screenBuffer, (255,255,255), ((positionHistory[i][0]/STATE_ETA[0]+1)*MAGNIFY,(positionHistory[i][1]/STATE_ETA[1]+1)*MAGNIFY),((positionHistory[i+1][0]/STATE_ETA[0]+1)*MAGNIFY,(positionHistory[i+1][1]/STATE_ETA[1]+1)*MAGNIFY),2)
        pygame.draw.circle(screenBuffer, (255,255,128), (int((positionHistory[-1][0]/STATE_ETA[0]+1)*MAGNIFY),int((positionHistory[-1][1]/STATE_ETA[1]+1)*MAGNIFY)), 5)

        # Flip!
        screen.blit(screenBuffer, (0, 0))
        # print "FPS: ",clock.get_fps()
        clock.tick(FPS)
        pygame.display.flip()


# ==================================
# Call main program
# ==================================
try:
    actionLoop()
except KeyboardInterrupt:
    os.kill(os.getpid(), signal.SIGKILL)
