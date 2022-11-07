import subprocess, math, time, sys, os, numpy as np
import matplotlib.pyplot as plt
import pybullet as bullet_simulation
import pybullet_data

# setup paths and load the core
abs_path = os.path.dirname(os.path.realpath(__file__))
root_path = abs_path + '/..'
core_path = root_path + '/core'
sys.path.append(core_path)
from Pybullet_Simulation import Simulation

# specific settings for this task

taskId = 3.2

try:
    if sys.argv[1] == 'nogui':
        gui = False
    else:
        gui = True
except:
    gui = True

pybulletConfigs = {
    "simulation": bullet_simulation,
    "pybullet_extra_data": pybullet_data,
    "gui": gui,
    "panels": False,
    "realTime": False,
    "controlFrequency": 1000,
    "updateFrequency": 250,
    "gravity": -9.81,
    "gravityCompensation": 1.,
    "floor": True,
    "cameraSettings": (1.2, 90, -22.8, (-0.12, -0.01, 0.99))
}
robotConfigs = {
    "robotPath": core_path + "/nextagea_description/urdf/NextageaOpen.urdf",
    "robotPIDConfigs": core_path + "/PD_gains.yaml",
    "robotStartPos": [0, 0, 0.85],
    "robotStartOrientation": [0, 0, 0, 1],
    "fixedBase": False,
    "colored": False
}

sim = Simulation(pybulletConfigs, robotConfigs)

##### Please leave this function unchanged, feel free to modify others #####
def getReadyForTask():
    global finalTargetPos
    global taleId, cubeId, targetId, obstacle
    finalTargetPos = np.array([0.35,0.38,1.0])
    # compile target urdf
    urdf_compiler_path = core_path + "/urdf_compiler.py"
    subprocess.call([urdf_compiler_path,
                     "-o", abs_path+"/lib/task_urdfs/task3_2_target_compiled.urdf",
                     abs_path+"/lib/task_urdfs/task3_2_target.urdf"])

    sim.p.resetJointState(bodyUniqueId=1, jointIndex=12, targetValue=-0.4)
    sim.p.resetJointState(bodyUniqueId=1, jointIndex=6, targetValue=-0.4)

    # load the table in front of the robot
    tableId = sim.p.loadURDF(
        fileName              = abs_path+"/lib/task_urdfs/table/table_taller.urdf",
        basePosition          = [0.8, 0, 0],             
        baseOrientation       = sim.p.getQuaternionFromEuler([0,0,math.pi/2]),                                  
        useFixedBase          = True,             
        globalScaling         = 1.4
    )
    cubeId = sim.p.loadURDF(
        fileName              = abs_path+"/lib/task_urdfs/cubes/task3_2_dumb_bell.urdf", 
        basePosition          = [0.5, 0, 1.1],            
        baseOrientation       = sim.p.getQuaternionFromEuler([0,0,0]),                                  
        useFixedBase          = False,             
        globalScaling         = 1.4
    )
    sim.p.resetVisualShapeData(cubeId, -1, rgbaColor=[1,1,0,1])
    
    targetId = sim.p.loadURDF(
        fileName              = abs_path+"/lib/task_urdfs/task3_2_target_compiled.urdf",
        basePosition          = finalTargetPos,             
        baseOrientation       = sim.p.getQuaternionFromEuler([0,0,math.pi/4]), 
        useFixedBase          = True,             
        globalScaling         = 1
    )
    obstacle = sim.p.loadURDF(
        fileName              = abs_path+"/lib/task_urdfs/cubes/task3_2_obstacle.urdf",
        basePosition          = [0.43,0.275,0.9],             
        baseOrientation       = sim.p.getQuaternionFromEuler([0,0,math.pi/4]), 
        useFixedBase          = True,             
        globalScaling         = 1
    )

    for _ in range(300):
        sim.tick([],[])
        time.sleep(1./1000)

    return tableId, cubeId, targetId

#TODO: Check for sim.tick() above what we are meant to pass. Currently passing [],[]. Comment above says to leave functions unmodified
# -> TypeError: Simulation.tick() missing 2 required positional arguments: 'targetStates' and 'targetJoints'
def solution():
    extraChestTrans =  np.array([0, 0, 0.267])
    goalLeft1 = np.array([0.5, 0.09, 1.07])   #Getting to pickup point 
    goalRight1 = np.array([0.5, -0.0, 1.07])  #Getting to pickup point
    

    translations = np.array([
        [0,0,0.1],
        [0,0,0.2],
        [0,0.15,0.2],
        [-0.10,0.35,0.2],
        [-0.15,0.39,0.2],
        [-0.15,0.39,0.08]
    ])
    goalLeft2 = translations + goalLeft1#Getting to drop point
    goalRight2 = translations + goalRight1#Getting to drop point

    
    shiftToAvoidTableCollision = np.array([0,0,0.075])
    startPointL = sim.getJointPosition("LARM_JOINT5") + shiftToAvoidTableCollision  + np.array([0, 0, 0.85]) 
    startPointR = sim.getJointPosition("RARM_JOINT5")  + shiftToAvoidTableCollision  + np.array([0, 0, 0.85])   

    # points_left = sim.cubic_interpolation(points_left, nTimes = 3)
    # points_right= sim.cubic_interpolation(points_right, nTimes = 3)
    #
    points_left = np.linspace(startPointL , goalLeft1 , 2)
    points_right= np.linspace(startPointR , goalRight1 , 2)
    print(points_right)
    for i in range(len(points_left)):
        p_l = points_left[i]
        p_r = points_right[i]

        # sim.move_with_PD("RARM_JOINT5", np.array(p_r) - np.array([0, 0, 0.85]) -extraChestTrans , speed=0.01, orientation=[0,-1,0], threshold=1e-3, maxIter=1000, debug=True, verbose=False, startJoint = "RARM_JOINT0")
        # sim.move_with_PD("LARM_JOINT5", np.array(p_l) -  np.array([0, 0, 0.85]) - extraChestTrans  , speed=0.01, orientation=[0,1,0], threshold=1e-3, maxIter=1000, debug=True, verbose=False, startJoint = "LARM_JOINT0")
        sim.move_with_PD_multiple( ["LARM_JOINT5", "RARM_JOINT5"], [np.array(p_l) - np.array([0, 0, 0.85]) ,
                                                                    np.array(p_r) - np.array([0, 0, 0.85])],
                                  speed=0.01, orientations=[[0,1,0], [0,-1,0]], threshold=1e-3, maxIter=1000, debug=True, verbose=False, startJoint = "")


    points_left =  goalLeft2
    points_right=  goalRight2
    points_left = sim.cubic_interpolation(points_left, nTimes = 15)
    points_right= sim.cubic_interpolation(points_right, nTimes = 15)

    for i in range(len(points_left)):
        p_l = points_left[i]
        p_r = points_right[i]

        sim.move_with_PD_multiple( ["LARM_JOINT5", "RARM_JOINT5"], [np.array(p_l) - np.array([0, 0, 0.85]) ,
                                                                    np.array(p_r) - np.array([0, 0, 0.85])],
                                  speed=0.01, orientations=[[0,1,0], [0,-1,0]], threshold=1e-3, maxIter=1000, debug=True, verbose=False, startJoint = "")



tableId, cubeId, targetId = getReadyForTask()
solution()