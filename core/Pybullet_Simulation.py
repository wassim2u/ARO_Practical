from concurrent.futures import thread
from multiprocessing.sharedctypes import Value
from tkinter import S
from scipy.spatial.transform import Rotation as npRotation
from scipy.special import comb
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np
import math
import re
import time
import yaml
import pybullet as p

from Pybullet_Simulation_base import Simulation_base

# TODO: Rename class name after copying this file
class Simulation(Simulation_base):
    """A Bullet simulation involving Nextage robot"""

    def __init__(self, pybulletConfigs, robotConfigs, refVect=None):
        """Constructor
        Creates a simulation instance with Nextage robot.
        For the keyword arguments, please see in the Pybullet_Simulation_base.py
        """
        super().__init__(pybulletConfigs, robotConfigs)
        if refVect:
            self.refVector = np.array(refVect)
        else:
            self.refVector = np.array([1,0,0])


        self.jointMap = {
            'base_to_dummy': 'base_to_waist',  # Virtual joint
            'base_to_waist': 'CHEST_JOINT0',  # Fixed joint
            # TODO: modify from here
            'CHEST_JOINT0_HEAD': 'HEAD_JOINT0',
            'CHEST_JOINT0_LEFT': 'LARM_JOINT0',
            'CHEST_JOINT0_RIGHT': 'RARM_JOINT0',
            'HEAD_JOINT0': 'HEAD_JOINT1',
            'HEAD_JOINT1': None,
            'LARM_JOINT0': 'LARM_JOINT1',
            'LARM_JOINT1': 'LARM_JOINT2',
            'LARM_JOINT2': 'LARM_JOINT3',
            'LARM_JOINT3': 'LARM_JOINT4',
            'LARM_JOINT4': 'LARM_JOINT5',
            'LARM_JOINT5': 'LARM',
            'RARM_JOINT0': 'RARM_JOINT1',
            'RARM_JOINT1': 'RARM_JOINT2',
            'RARM_JOINT2': 'RARM_JOINT3',
            'RARM_JOINT3': 'RARM_JOINT4',
            'RARM_JOINT4': 'RARM_JOINT5',
            'RARM_JOINT5': 'RARM',
            #'RHAND'      : None, # optional
            #'LHAND'      : None # optional
        }

        ########## Task 1: Kinematics ##########
        # Task 1.1 Forward Kinematics
        self.jointRotationAxis = {
            'base_to_dummy': np.zeros(3),  # Virtual joint
            'base_to_waist': np.zeros(3),  # Fixed joint
            # TODO: modify from here
            'CHEST_JOINT0': np.array([0, 0, 1]),
            'HEAD_JOINT0': np.array([0, 0, 1]),
            'HEAD_JOINT1': np.array([0, 1, 0]),
            'LARM_JOINT0': np.array([0, 0, 1]),
            'LARM_JOINT1': np.array([0, 1, 0]),
            'LARM_JOINT2': np.array([0, 1, 0]),
            'LARM_JOINT3': np.array([1, 0, 0]),
            'LARM_JOINT4': np.array([0, 1, 0]),
            'LARM_JOINT5': np.array([0, 0, 1]),
            'RARM_JOINT0': np.array([0, 0, 1]),
            'RARM_JOINT1': np.array([0, 1, 0]),
            'RARM_JOINT2': np.array([0, 1, 0]),
            'RARM_JOINT3': np.array([1, 0, 0]),
            'RARM_JOINT4': np.array([0, 1, 0]),
            'RARM_JOINT5': np.array([0, 0, 1]),
            #'RHAND'      : np.array([0, 0, 0]),
            #'LHAND'      : np.array([0, 0, 0])
        }

        self.frameTranslationFromParent = {
            'base_to_dummy': np.zeros(3),  # Virtual joint
            'base_to_waist': np.zeros(3),  # Fixed joint
            # TODO: modify from here
            'CHEST_JOINT0': np.array([0, 0, 0.267]),
            'HEAD_JOINT0': np.array([0, 0, 0.302]),
            'HEAD_JOINT1': np.array([0, 0, 0.066]),
            'LARM_JOINT0': np.array([0.04, 0.135, 0.1015]),
            'LARM_JOINT1': np.array([0, 0, 0.066]),
            'LARM_JOINT2': np.array([0, 0.095, -0.25]),
            'LARM_JOINT3': np.array([0.1805, 0, -0.03]),
            'LARM_JOINT4': np.array([0.1495, 0, 0]),
            'LARM_JOINT5': np.array([0, 0, -0.1335]),
            'RARM_JOINT0': np.array([0.04, -0.135, 0.1015]),
            'RARM_JOINT1': np.array([0, 0, 0.066]),
            'RARM_JOINT2': np.array([0, -0.095, -0.25]),
            'RARM_JOINT3': np.array([0.1805, 0, -0.03]),
            'RARM_JOINT4': np.array([0.1495, 0, 0]),
            'RARM_JOINT5': np.array([0, 0, -0.1335]),
            #'RHAND'      : np.array([0, 0, 0]), # optional
            #'LHAND'      : np.array([0, 0, 0]) # optional
        }

    def measureJointAngles(self):
        jointPos = {}
        for key in self.jointRotationAxis.keys():
            jointPos[key] = self.getJointPos(key)
        return jointPos

    def calculateTransformationMatrices(self, jointPos):
        transformationMatrices = {}
        for key, val in self.frameTranslationFromParent.items():
            theta = jointPos[key]
            R = self.getJointRotationalMatrix(key, theta)
            
            R = np.vstack((R, np.array([[0,0,0]])))
            R = np.hstack((R, np.transpose(np.array([np.append(val, 1)]))))
            transformationMatrices[key] = R


        return transformationMatrices
        
        
    def getNextJoint(self, jointName, finalJoint):
        if "CHEST_JOINT0" in jointName and "LARM" in finalJoint:
            return self.jointMap["CHEST_JOINT0_LEFT"]
        elif "CHEST_JOINT0" in jointName and "RARM" in finalJoint:
            return self.jointMap["CHEST_JOINT0_RIGHT"]
        elif "CHEST_JOINT0" in jointName and "HEAD" in finalJoint:
            return self.jointMap["CHEST_JOINT0_HEAD"]
        elif "base_to_waist" == jointName:
            return "CHEST_JOINT0"          
        else:
           
            nextJoint =  self.jointMap[jointName]
            if nextJoint is None:
                return jointName
            else:
                return nextJoint

    def getJointRotationalMatrix(self, jointName=None, theta=None):
        """
            Returns the 3x3 rotation matrix for a joint from the axis-angle representation,
            where the axis is given by the revolution axis of the joint and the angle is theta.
        """
        if jointName == None:
            raise Exception("[getJointRotationalMatrix] \
                Must provide a joint in order to compute the rotational matrix!")
        # TODO modify from here
        # Hint: the output should be a 3x3 rotational matrix as a numpy array
        #return np.matrix()
        rotationAxis = self.jointRotationAxis[jointName]

        cos = np.cos(theta)
        sin = np.sin(theta)
        n_0 = rotationAxis[0]
        n_1 = rotationAxis[1]
        n_2 = rotationAxis[2]

        R =     np.array([  [cos+(n_0**2)*(1-cos),       n_0*n_1*(1-cos)-n_2*sin,   n_0*n_2*(1-cos) + n_1*sin   ],
                            [n_0*n_1*(1-cos) + n_2*sin,  cos+(n_1**2)*(1-cos),      n_1*n_2*(1-cos)-n_0*sin     ],
                            [n_0*n_2*(1-cos)-n_1*sin,    n_1*n_2*(1-cos) + n_0*sin, cos+(n_2**2)*(1-cos)        ]])
        
        # R_x = np.array([[1, 0,              0             ],
        #                 [0, np.cos(theta),  -np.sin(theta)],
        #                 [0, np.sin(theta),  np.cos(theta) ]])

        # R_y = np.array([[np.cos(theta),  0, np.sin(theta)],
        #                 [0,              1, 0            ],
        #                 [-np.sin(theta), 0, np.cos(theta)]])

        # R_z = np.array([[np.cos(theta), -np.sin(theta), 0],
        #                 [np.sin(theta), np.cos(theta),  0],
        #                 [0,             0,              1]])
        

        # rotationAxis = self.jointRotationAxis[jointName]
        # R = np.identity(3)
        # #print(rotationAxis)
        # #if not np.any(rotationAxis):
        # #     return np.zeros((3,3))
        # if(rotationAxis[2]):
        #     R = R @ R_z
        # elif(rotationAxis[1]):
        #     R = R @ R_y
        # elif(rotationAxis[0]):
        #     R = R @ R_x
        # else:
        #     R = np.zeros((3,3))

        return R

    def getTransformationMatrices(self):
        """
            Returns the homogeneous transformation matrices for each joint as a dictionary of matrices.
        """
        transformationMatrices = {}
        # TODO modify from here
        # Hint: the output should be a dictionary with joint names as keys and
        # their corresponding homogeneous transformation matrices as values.
        for key, val in self.frameTranslationFromParent.items():
            theta = self.getJointPos(key)
            R = self.getJointRotationalMatrix(key, theta)
            
            R = np.vstack((R, np.array([[0,0,0]])))
            R = np.hstack((R, np.transpose(np.array([np.append(val, 1)]))))


            
            transformationMatrices[key] = R
        return transformationMatrices
    
    
    def forwardKinematics(self, jointName, jointPos, startJoint="base_to_dummy"):
        """
            Returns the position and rotation matrix of a given joint using Forward Kinematics
            according to the topology of the Nextage robot.
        """
        htms = self.calculateTransformationMatrices(jointPos)
        

        fkMatrices = []
        jointNames = []
        # current joint and its transformation matrix
        htm = htms[startJoint]
        fkMatrices.append(htm)
        jointNames.append(startJoint)
        nextJoint = self.getNextJoint(startJoint, jointName)

        while nextJoint != jointName:

            htm =  htm @htms[nextJoint] 
            fkMatrices.append(htm)
            jointNames.append(nextJoint)

            nextJoint = self.getNextJoint(nextJoint, jointName)

        htm =  htm @htms[nextJoint] 
        fkMatrices.append(htm)
        jointNames.append(nextJoint)
        #print(jointNames)
        

        return fkMatrices, jointNames

    def getJointLocationAndOrientation(self, jointName):
        """
            Returns the position and rotation matrix of a given joint using Forward Kinematics
            according to the topology of the Nextage robot.
        """

        jointMatrices, _ = self.forwardKinematics(jointName, self.measureJointAngles())
        jointMatrix = jointMatrices[-1]
        p_i, a_i = self.extractPositionAndAngle(jointMatrix)

        return p_i, a_i
    
        
        # Remember to multiply the transformation matrices following the kinematic chain for each arm.
        #TODO modify from here
        # Hint: return two numpy arrays, a 3x1 array for the position vector,
        # and a 3x3 array for the rotation matrix
        # return pos, rotmat
    

    def getJointPosition(self, jointName):
        """Get the position of a joint in the world frame, leave this unchanged please."""
        return self.getJointLocationAndOrientation(jointName)[0]

    def getJointOrientation(self, jointName, ref=None):
        """Get the orientation of a joint in the world frame, leave this unchanged please."""
        if ref is None:
            return np.array(self.getJointLocationAndOrientation(jointName)[1] @ self.refVector).squeeze()
        else:
            return np.array(self.getJointLocationAndOrientation(jointName)[1] @ ref).squeeze()

    def getJointAxis(self, jointName):
        """Get the orientation of a joint in the world frame, leave this unchanged please."""
        return np.array(self.getJointLocationAndOrientation(jointName)[1] @ self.jointRotationAxis[jointName]).squeeze()

    def jacobianMatrix(self, endEffector, fkMatrices, jointNames):
        """Calculate the Jacobian Matrix for the Nextage Robot."""
        # TODO modify from here
        # You can implement the cross product yourself or use calculateJacobian().
        # Hint: you should return a numpy array for your Jacobian matrix. The
        # size of the matrix will depend on your chosen convention. You can have
        # a 3xn or a 6xn Jacobian matrix, where 'n' is the number of joints in
        # your kinematic chain.
        #return np.array()
        
        #print("End effector " + str(endEffector) + "\n")
        #print(self.jointRotationAxis[endEffector])
        #print(self.getJointLocationAndOrientation(endEffector))
        #Retrieve the necessary values from end effector 
        
        # p_eff = self.getJointPosition(endEffector)
        # a_eff = self.getJointAxis(endEffector)
        p_eff, a_eff = self.extractPositionAndAngle(fkMatrices[-1])
        a_eff = a_eff@(self.jointRotationAxis[endEffector]) 
        assert(self.getJointPosition(endEffector), p_eff)
        assert(self.getJointAxis(endEffector), a_eff)
        #TODO: Calculate the transformation matrix once. There may be a better way to go through the homogenous matrix keys
        #Initialise the matrix to be filled
        jacobian = []
        

        for idx, jointMatrix in enumerate(fkMatrices):
            #Ensure we dont compute the same end effector. 
            
            p_i, a_i = self.extractPositionAndAngle(jointMatrix)
            a_i = a_i@(self.jointRotationAxis[jointNames[idx]])
            
            jacobian_position = np.cross(a_i, p_eff - p_i)
            jacobian_vector= np.cross(a_i, a_eff) 

            jacobian.append(np.hstack((jacobian_position, jacobian_vector)))
            #jacobian.append(jacobian_position)

        return np.array(jacobian).T
    
    ################# Helper Functions ################
    
    def extractPositionAndAngle(self, jointMatrix):
        a_i = jointMatrix[:3,:3]

        p_i = jointMatrix[:3,3]

        return p_i, a_i
        
        
    # Task 1.2 Inverse Kinematics

    def inverseKinematics(self, endEffector, targetPosition, orientation, interpolationSteps, threshold, startJoint):
        """Your IK solver \\
        Arguments: \\
            endEffector: the jointName the end-effector \\
            targetPosition: final destination the the end-effector \\
            orientation: the desired orientation of the end-effector
                         together with its parent link \\
            interpolationSteps: number of interpolation steps
            maxIterPerStep: maximum iterations per step
            threshold: accuracy threshold
        Return: \\
            Vector of x_refs
        """
        # TODO add your code here
        # Hint: return a numpy array which includes the reference angular
        # positions for all joints after performing inverse kinematics.
        if orientation==None:
            orientation = [0,0,0]
            # orientation = self.getJointOrientation(endEffector) 
            print(self.refVector)
        #TODO: Work with orientation as well
        
        # set the initial robot configuration
        # dTheta = initTheta

        jointAngles = self.measureJointAngles()

        #FK
        fkMatrices, jointNames = self.forwardKinematics(endEffector, jointAngles, startJoint)
        #print(jointNames)
        #print(len(fkMatrices))
        #print(len(jointNames))
        efPosition, efRotationMatrix = self.extractPositionAndAngle(fkMatrices[-1])
        
        efOrientation = self.getJointOrientation(endEffector) 
        print(efOrientation)
        #Joint angles
        q = np.array([ jointAngles[val] for val in jointNames] ) 
 
        traj = [q]
        EFDif = [np.linalg.norm(efPosition - targetPosition)]
        EFLocations = [efPosition]
        
        # -- Debug - Draw where the end effector starts in light blue
        print("Starting EF Position :" + str(efPosition))
        print("Starting EF Angle :" + str(efAngle))
        visualShift = efPosition
        collisionShift = [0,0,0]
        inertiaShift = [0,0,0]

        meshScale=[0.1,0.1,0.1]
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[0,1,1,1],radius= 0.02, visualFramePosition=visualShift, meshScale=meshScale)

        p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)
        # Compute the tiny changes in positions for the end effector to go towards the target
        stepPositions = np.linspace(efPosition,targetPosition,num=interpolationSteps)
        #TODO: Check whether we are meant to retrieve step orientations with linspace. Sti
        stepOrientations = np.linspace(efOrientation,orientation,num=interpolationSteps)
        for i in range(len(stepPositions)):
            currGoal = stepPositions[i]

            dy = currGoal - efPosition
            dyOrientation = orientation - efAngle@self.jointRotationAxis[endEffector]
            dy = np.hstack((dy, dyOrientation))


            jacobian = self.jacobianMatrix(endEffector, fkMatrices, jointNames=jointNames)

            dq = np.linalg.pinv(jacobian)@dy

            q += dq
            #print(dq)
            
            traj.append(q)
            
            for i in range(len(dq)):
                jointAngles[jointNames[i]] = q[i]

            
            q = np.array([ jointAngles[val] for val in jointNames] ) 
            
            #Calculate the FK again with the updated joint angles
            fkMatrices, jointNames_2 = self.forwardKinematics(endEffector, jointAngles, startJoint)

            assert(jointNames== jointNames_2)
            #Calculate the new end effector position
            efPosition, efAngle = self.extractPositionAndAngle(fkMatrices[-1])
            # EFLocations.append(efPosition)
            
                
            
            
            #TODO: calculate the new end effector 
            #Missing the FK calculate and updating end effector 
            #if np.linalg.norm(efPosition - currGoal) < threshold:
            #    break
            EFLocations.append(efPosition)
            visualShift = efPosition
            collisionShift = [0,0,0]
            inertiaShift = [0,0,0]

            meshScale=[0.1,0.1,0.1]
            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[0,1,0,1],radius= 0.005, visualFramePosition=visualShift, meshScale=meshScale)

            p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)

            EFDif.append(np.linalg.norm(efPosition - targetPosition))
            

        #TODO: You should directly (re)set the joint positions to be the desired values using a method such as Simulation.p.resetJointState() or else
        
        print(efOrientation)
        return np.array(traj), jointNames, EFDif, EFLocations

    def move_without_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False, startJoint = "base_to_dummy"):
        """
        Move joints using Inverse Kinematics solver (without using PD control).
        This method should update joint states directly.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        #TODO add your code here
        # iterate through joints and update joint states based on IK solver
        trajs, names, EFPositions, _ = self.inverseKinematics(endEffector=endEffector, targetPosition=targetPosition, 
                               orientation=orientation,
                               interpolationSteps=50, #TODO: whats the  interpolation step here?
                               threshold=threshold,
                               startJoint=startJoint
                               )

        #return pltTime, pltDistance

        for traj in trajs:
            self.tick_without_PD(names, traj)
            
        pltTimes = [i for i in range(len(EFPositions))]

        return pltTimes, EFPositions

    def tick_without_PD(self, names, traj):
        """Ticks one step of simulation without PD control. """
        # TODO modify from here
        # Iterate through all joints and update joint states.
            # For each joint, you can use the shared variable self.jointTargetPos.
            
        for i, name in enumerate(names):                
                self.p.resetJointState(self.robot, self.jointIds[name], traj[i])
                
        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)


    ########## Task 2: Dynamics ##########
    # Task 2.1 PD Controller
    def calculateTorque(self, x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd):
        """ This method implements the closed-loop control \\
        Arguments: \\
            x_ref - the target position \\
            x_real - current position \\
            dx_ref - target velocity \\
            dx_real - current velocity \\
            integral - integral term (set to 0 for PD control) \\
            kp - proportional gain \\
            kd - derivetive gain \\
            ki - integral gain \\
        Returns: \\
            u(t) - the manipulation signal
        """
        integral = 0
        # proportional term
        u_t = kp * (x_ref - x_real) + kd * (dx_ref - dx_real)  + ki * integral
        # TODO: Add your code here
        return u_t

    # Task 2.2 Joint Manipulation
    def moveJoint(self, joint, targetPosition, targetVelocity, verbose=False):
        """ This method moves a joint with your PD controller. \\
        Arguments: \\
            joint - the name of the joint \\
            targetPos - target joint position \\
            targetVel - target joint velocity
        """
        if('pos' not in self.jointsInfos[joint]):
            self.jointsInfos[joint]['pos'] = self.getJointPos(joint)
        if('vel' not in self.jointsInfos[joint]):
            self.jointsInfos[joint]['vel'] = 0
        self.jointsInfos[joint]['lastPos']   = self.jointsInfos[joint]['pos']
        self.jointsInfos[joint]['pos']       = self.getJointPos(joint)
        
        self.jointsInfos[joint]['lastVel']   = self.jointsInfos[joint]['vel']
        self.jointsInfos[joint]['vel']       = (self.jointsInfos[joint]['pos'] - self.jointsInfos[joint]['lastPos']) / self.dt

        def toy_tick(x_ref, x_real, dx_ref, dx_real, integral):
            # loads your PID gains
            jointController = self.jointControllers[joint]
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Start your code here: ###
            # Calculate the torque with the above method you've made
            torque = self.calculateTorque(x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd)
            
            ### To here ###

            pltTorque.append(torque)

            #print("TORQUE:", torque)

            # send the manipulation signal to the joint
            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )
            # calculate the physics and update the world
            self.p.stepSimulation()
            time.sleep(self.dt)

        targetPosition, targetVelocity = float(targetPosition), float(targetVelocity)
        
        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)

        
        # logging for the graph
        pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = [], [], [], [], [], []
        
        while (abs(self.getJointPos(joint) - targetPosition) > 1e-3):
            toy_tick(targetPosition, self.jointsInfos[joint]['pos'], targetVelocity, self.jointsInfos[joint]['vel'], integral=0)
            
        return pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity

    def move_with_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False, startJoint ="base_to_dummy"):
        """
        Move joints using inverse kinematics solver and using PD control.
        This method should update joint states using the torque output from the PD controller.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        targetStatess, jointNames, efDiffs, eflocations = self.inverseKinematics(endEffector=endEffector, targetPosition=targetPosition, 
                                                                orientation=orientation,
                                                                interpolationSteps=20, #TODO: whats the  interpolation step here?
                                                                threshold=threshold,
                                                                startJoint=startJoint
                                                                )
                        
        print("Done with kinematics")

        final = targetStatess[-1]


        # for i, targetStates in enumerate(targetStatess):
            
        visualShift = eflocations[-1]
        collisionShift = [0,0,0]
        inertiaShift = [0,0,0]

        meshScale=[0.1,0.1,0.1]
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1,0,0,1],radius= 0.015, visualFramePosition=visualShift, meshScale=meshScale)

        p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)

        for j in range(maxIter):
    #while(np.linalg.norm(self.getJointPosition(endEffector) - targetPosition) > threshold):
        
            self.tick(targetStatess[-1], jointNames)
            #print(np.linalg.norm(self.getJointPosition(endEffector) -  eflocations[-1]))#eflocations[i]))
            if(np.linalg.norm(self.getJointPosition(endEffector) -  eflocations[-1]) < threshold):
                print("BREAK YOUR KNEES")
                break
                
            #print()
            #   
        print("DONE")
        
        #TODO add your code here
        # Iterate through joints and use states from IK solver as reference states in PD controller.
        # Perform iterations to track reference states using PD controller until reaching
        # max iterations or position threshold.

        # Hint: here you can add extra steps if you want to allow your PD
        # controller to converge to the final target position after performing
        # all IK iterations (optional).
        
        

        # return pltTime, pltDistance
        return

    def tick(self, targetStates, targetJoints):
        """Ticks one step of simulation using PD control."""
        # Iterate through all joints and update joint states using PD control.
        for i, joint in enumerate(targetJoints):
            # skip dummy joints (world to base joint)
            jointController = self.jointControllers[joint]
            if jointController == 'SKIP_THIS_JOINT':
                continue
            targetState = targetStates[i]
            # disable joint velocity controller before apply a torque
            self.disableVelocityController(joint)
    
            # loads your PID gains
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Implement your code from here ... ###
            # TODO: obtain torque from PD controller
            if('pos' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['pos'] = self.getJointPos(joint)
            if('vel' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['vel'] = 0
            
            self.jointsInfos[joint]['lastPos']   = self.jointsInfos[joint]['pos']
            self.jointsInfos[joint]['pos']       = self.getJointPos(joint)
            
            self.jointsInfos[joint]['lastVel']   = self.jointsInfos[joint]['vel']
            self.jointsInfos[joint]['vel']       = (self.jointsInfos[joint]['pos'] - self.jointsInfos[joint]['lastPos']) / self.dt
            torque = self.calculateTorque(  targetState, 
                                            self.jointsInfos[joint]['pos'],
                                            0, 
                                            self.jointsInfos[joint]['vel'],
                                            0, kp, ki, kd)  # TODO: fix me
            ### ... to here ###
            #print(joint, ":", torque)
            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )

            # Gravity compensation
            # A naive gravitiy compensation is provided for you
            # If you have embeded a better compensation, feel free to modify
            compensation = self.jointGravCompensation[joint]
            self.p.applyExternalForce(
                objectUniqueId=self.robot,
                linkIndex=self.jointIds[joint],
                forceObj=[0, 0, -compensation],
                posObj=self.getLinkCoM(joint),
                flags=self.p.WORLD_FRAME
            )
            # Gravity compensation ends here

        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)


    ########## Task 3: Robot Manipulation ##########
    def cubic_interpolation(self, x, y, nTimes=100):
        """
        Given a set of control points, return the
        cubic spline defined by the control points,
        sampled nTimes along the curve.
        """
        cs = CubicSpline(x, y, bc_type = "clamped")
        points = list(zip(*[cs(i) for i in range(nTimes)]))

        #TODO add your code here
        # Return 'nTimes' points per dimension in 'points' (typically a 2xN array),
        # sampled from a cubic spline defined by 'points' and a boundary condition.
        # You may use methods found in scipy.interpolate

        return points[0], points[1]

    # Task 3.1 Pushing
    def dockingToPosition(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005,
            threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

    # Task 3.2 Grasping & Docking
    def clamp(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005, threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

 ### END
