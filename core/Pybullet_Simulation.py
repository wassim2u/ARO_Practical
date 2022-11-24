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
            'LARM_JOINT5': None, 
            'RARM_JOINT0': 'RARM_JOINT1',
            'RARM_JOINT1': 'RARM_JOINT2',
            'RARM_JOINT2': 'RARM_JOINT3',
            'RARM_JOINT3': 'RARM_JOINT4',
            'RARM_JOINT4': 'RARM_JOINT5',
            'RARM_JOINT5': None
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
        """ Retrieve the revolute positions of all joints into a dictionary

        Returns:
            jointPos (dict): dictionary of joint names mapped to the angle joints in radians 
        """
        jointPos = {}
        for key in self.jointRotationAxis.keys():
            jointPos[key] = self.getJointPos(key)
        return jointPos

    def calculateTransformationMatrices(self, jointPos):
        """ Returns the homogeneous transformation matrices for each joint as a dictionary of matrices.
        i.e) Assume we simplify our robot model to one arm joints of A,B,C with 0 as the origin, we define the transformation matrix T for A as T_{0->A}, for B as T_{A->B}, and C as T_{B->C}.
        
        Args:
            jointPos (dict): Contains the joints current angles in radians mapped to its respective joint in the dictionary.
        Returns:
            transformationMatrices (dict) : dictionary of joint transformation matricies T for all the joints defined in the robot.
        """
        transformationMatrices = {}
        for key, val in self.frameTranslationFromParent.items():
            theta = jointPos[key]
            R = self.getJointRotationalMatrix(key, theta)
            
            R = np.vstack((R, np.array([[0,0,0]])))
            R = np.hstack((R, np.transpose(np.array([np.append(val, 1)]))))
            transformationMatrices[key] = R


        return transformationMatrices
        

    def getNextJoint(self, jointName, finalJointName):
        """Gets the next consecutive joint of the robot. If the current given joint is a chest joint, we rely on the target joint to help determine the correct successor.
        This allows us to piece together the kinematics chain for the computation of Forward Kinematics.
        
        Args:
            jointName (str): represents the current joint 
            finalJointName (str): represents the final joint.
        Returns:
            str: successor joint name, if any.
        """
        if "CHEST_JOINT0" in jointName and "LARM" in finalJointName:
            return self.jointMap["CHEST_JOINT0_LEFT"]
        elif "CHEST_JOINT0" in jointName and "RARM" in finalJointName:
            return self.jointMap["CHEST_JOINT0_RIGHT"]
        elif "CHEST_JOINT0" in jointName and "HEAD" in finalJointName:
            return self.jointMap["CHEST_JOINT0_HEAD"]
        elif "base_to_waist" == jointName:
            return "CHEST_JOINT0"          
        else:
            
            nextJoint =  self.jointMap[jointName]
            return nextJoint

    def getJointRotationalMatrix(self, jointName=None, theta=None):
        """
            Returns the 3x3 rotation matrix for a joint from the axis-angle representation,
            where the axis is given by the revolution axis of the joint and the angle is theta.
        """
        if jointName == None:
            raise Exception("[getJointRotationalMatrix] \
                Must provide a joint in order to compute the rotational matrix!")
        # Hint: the output should be a 3x3 rotational matrix as a numpy array
        rotationAxis = self.jointRotationAxis[jointName]

        cos = np.cos(theta)
        sin = np.sin(theta)
        n_0 = rotationAxis[0] # - first element of the rotation axis
        n_1 = rotationAxis[1] # - second element of the rotation axis
        n_2 = rotationAxis[2] # - third element of the rotation axis

        R =     np.array([  [cos+(n_0**2)*(1-cos),       n_0*n_1*(1-cos)-n_2*sin,   n_0*n_2*(1-cos) + n_1*sin   ],
                            [n_0*n_1*(1-cos) + n_2*sin,  cos+(n_1**2)*(1-cos),      n_1*n_2*(1-cos)-n_0*sin     ],
                            [n_0*n_2*(1-cos)-n_1*sin,    n_1*n_2*(1-cos) + n_0*sin, cos+(n_2**2)*(1-cos)        ]])
        
        return R

    def getTransformationMatrices(self):
        """
            Returns the homogeneous transformation matrices for each joint as a dictionary of matrices.
        """
        transformationMatrices = {}
        jointAngles = self.measureJointAngles()
        left, jointnamesleft = self.forwardKinematics("LARM_JOINT5", jointAngles)
        right,jointnamesright = self.forwardKinematics("RARM_JOINT5", jointAngles)
        head, jointnameshead = self.forwardKinematics("HEAD_JOINT1", jointAngles)
        # some overlap but its fine
        for i, name in enumerate(jointnamesleft):
            transformationMatrices[name] = left[i]
        for i, name in enumerate(jointnamesright):
            transformationMatrices[name] = right[i]
        for i, name in enumerate(jointnameshead):
            transformationMatrices[name] = head[i]

        return transformationMatrices
        
    def forwardKinematics(self, finalJointName, jointPos, startJoint="base_to_dummy"):
        """
        Compute the forward kinematics for all joints in the kinematic chain up until the joint of interest which is the end effector.

        Args:
            finalJointName (str): The name of the end effector, which would be our final joint that we are interested in its forward kinematics..
            jointPos (dict): The dictionary of joints to their revolute joint positions in radians 
            startJoint (str, optional): The starting joint of where we want to compute the FK from. Defaults to "base_to_dummy" joint.

        Returns:  (list,list)
            fkMatrices: list of the resulting forward kinematics for each joint.
            jointNames: list of joint names whose forward kinematics are computed. The order of this list corresponds respectively to the list of forward kinematic matricies, 
                            with the end effector joint being at the end.
        """
        fkMatrices = [] # List of computed forward kinematics, for each joint that are part of the kinematic chain to the final joint
        jointNames = [] # List of joint names that are part of the kinematic chain to the final joint
        htms = self.calculateTransformationMatrices(jointPos) #Calculate the homogenous transformation matricies
        
        # Include the start joint as current joint and its transformation matrix
        htm = htms[startJoint]
        fkMatrices.append(htm)
        jointNames.append(startJoint)
        # get next joint in kinematic chain 
        nextJoint = self.getNextJoint(startJoint, finalJointName)
        while nextJoint != finalJointName:
            #Compute FK for the joint as a result of homogenous matrix multiplications of all the previous joints up to the current joint
            htm =  htm @htms[nextJoint] 
            fkMatrices.append(htm)
            jointNames.append(nextJoint)
            # get next joint in kinematic chain 
            nextJoint = self.getNextJoint(nextJoint, finalJointName)

        # Include the final specified joint name's FK
        htm =  htm @htms[nextJoint] 
        fkMatrices.append(htm)
        jointNames.append(nextJoint)
        
        return fkMatrices, jointNames

    def getJointLocationAndOrientation(self, jointName):
        """
            Returns the position and rotation matrix of a given joint using Forward Kinematics
            according to the topology of the Nextage robot.
        """

        # do FK, and extract position and rotation from the last link in the chain
        jointMatrices, _ = self.forwardKinematics(jointName, self.measureJointAngles())
        jointMatrix = jointMatrices[-1]
        p_i, r_i = self.extractPositionAndRotation(jointMatrix)

        return p_i, r_i


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
        """Calculate the Jacobian Matrix for the Nextage Robot.

        Args:
            endEffector (str): end effector to compute the jacobian matrix for.
            fkMatrices (list): list of forward kinematics for each joint (that is part of the kinematic chain to the end effector)
            jointNames (list): list of joint names

        Returns:
            ndarray: 6xN jacobian matrix, where N is the number of joints determined by the given arguments of the kinematic chain (i.e fkMatrices or jointNames)
                     and our joint of interest i.e) end effector.
        """
        
        # Retrieve the position and rotation axes of the end effector
        p_eff, r_eff = self.extractPositionAndRotation(fkMatrices[-1])
        a_eff = r_eff@(self.jointRotationAxis[endEffector])  #Retrieves rotation axes

        jacobian = []
        
        # for each joint i, calculate its position and vector jacobian.
        for i, jointMatrix in enumerate(fkMatrices):
            
            p_i, r_i = self.extractPositionAndRotation(jointMatrix)
            a_i = r_i@(self.jointRotationAxis[jointNames[i]])
            
            jacobian_position = np.cross(a_i, p_eff - p_i) # Calculate the position increment
            jacobian_vector= np.cross(a_i, a_eff)  # Calculate the orientation increment

            jacobian.append(np.hstack((jacobian_position, jacobian_vector)))
       
        # results in 6xN matrix 
        return np.array(jacobian).T

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

    
    
    ################# Helper Functions ################
    
    def extractPositionAndRotation(self, jointMatrix):
        """Extracts the 3x1 translation vector and the 3x3 rotation matrix from the 4x4 homogenous transformation matrix

        Args:
            jointMatrix (ndarray): 4x4 homogenous transformation matrix of a joint

        Returns: (ndarray, ndarray)
            p_i: 3x1 position matrix
            r_i: 3x3 rotation matrix
        """
        p_i = jointMatrix[:3,3]
        r_i = jointMatrix[:3,:3]
        return p_i, r_i
        
        
    # Task 1.2 Inverse Kinematics

    def inverseKinematics(self, endEffector, targetPosition, orientation, interpolationSteps, threshold, startJoint = "base_to_dummy", debug=False):
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
        timePassed = 0

        if orientation==None:
            orientation = [0,0,0]
            # orientation = self.getJointOrientation(endEffector) 
            print(self.refVector)

        jointAngles = self.measureJointAngles()

        fkMatrices, jointNames = self.forwardKinematics(endEffector, jointAngles, startJoint)

        efPosition, efAngle = self.extractPositionAndRotation(fkMatrices[-1])
        
        efOrientation = self.getJointAxis(endEffector) 
        #print(efOrientation)
        
        #Joint angles
        q = np.array([ jointAngles[val] for val in jointNames] ) 
 
        traj = [q]
        EFDif = [np.linalg.norm(efPosition - targetPosition)]
        EFLocations = [efPosition]
        pltTimes =[0]
        # -- Debug - Draw where the end effector starts in light blue
        #print("Starting EF Position :" + str(efPosition))
        #print("Starting EF Angle :" + str(efAngle))
        if debug:
            
            visualShift = efPosition
            inertiaShift = [0,0,0]

            meshScale=[0.1,0.1,0.1]
            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[0,1,1,1],radius= 0.02, visualFramePosition=visualShift, meshScale=meshScale)

            p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)
        # Compute the tiny changes in positions for the end effector to go towards the target
        stepPositions = np.linspace(efPosition,targetPosition,num=interpolationSteps)

        for i in range(len(stepPositions)):
            currGoal = stepPositions[i]

            # diff for both position and angle
            dy = currGoal - efPosition
            dyOrientation = orientation - efAngle@self.jointRotationAxis[endEffector]
            dy = np.hstack((dy, dyOrientation))                

            jacobian = self.jacobianMatrix(endEffector, fkMatrices, jointNames=jointNames)
            #print("JACOBIAN SHAPE")
            #print(jacobian.shape)
            #print(jointNames)

            # Moore-Penrose pseudoinverse
            dq = np.linalg.pinv(jacobian)@dy

            q += dq
            
            traj.append(q)
            
            for i in range(len(dq)):
                jointAngles[jointNames[i]] = q[i]

            q = np.array([ jointAngles[val] for val in jointNames] ) 
            
            #Calculate the FK again with the updated joint angles
            fkMatrices, jointNames_2 = self.forwardKinematics(endEffector, jointAngles, startJoint)

            assert(jointNames== jointNames_2)
            #Calculate the new end effector position
            efPosition, efAngle = self.extractPositionAndRotation(fkMatrices[-1])
            efOrientation = self.getJointAxis(endEffector) 
            
            # EFLocations.append(efPosition)
            
            
            #TODO: calculate the new end effector 
            #Missing the FK calculate and updating end effector 
            #if np.linalg.norm(efPosition - currGoal) < threshold:
            #    break
            EFLocations.append(efPosition)
            
            if debug:
                visualShift = efPosition
                collisionShift = [0,0,0]
                inertiaShift = [0,0,0]

                meshScale=[0.1,0.1,0.1]
                visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[0,1,0,1],radius= 0.005, visualFramePosition=visualShift, meshScale=meshScale)

                p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)

            EFDif.append(np.linalg.norm(efPosition - targetPosition))
            timePassed += self.dt
            pltTimes.append(timePassed)
            
        
        #print(efOrientation)
        return np.array(traj), jointNames, EFDif, EFLocations, pltTimes



    def move_without_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False, startJoint = "base_to_dummy"):
        """
        Move joints using Inverse Kinematics solver (without using PD control).
        This method should update joint states directly.
        Return:
            pltTime, pltDistance arrays used for plotting
        """

        trajs, names, EFPositions, _, pltTime = self.inverseKinematics(endEffector=endEffector, targetPosition=targetPosition, 
                               orientation=orientation,
                               interpolationSteps=50, 
                               threshold=threshold,
                               startJoint=startJoint
                               )
        for traj in trajs:
            self.tick_without_PD(names, traj)
            

        return pltTime, EFPositions

    def tick_without_PD(self, names, traj):
        """Ticks one step of simulation without PD control. """
        
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
        return u_t

    # Task 2.2 Joint Manipulation
    def moveJoint(self, joint, targetPosition, targetVelocity, verbose=False):
        """ This method moves a joint with your PD controller. \\
        Arguments: \\
            joint - the name of the joint \\
            targetPos - target joint position \\
            targetVel - target joint velocity
        """
        
        
        def toy_tick(x_ref, x_real, dx_ref, dx_real, integral):
            # loads your PID gains
            jointController = self.jointControllers[joint]
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']


            torque = self.calculateTorque(x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd)
            
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
            return torque

        targetPosition, targetVelocity = float(targetPosition), float(targetVelocity)
        
        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)

        
        # logging for the graph
        pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = [], [], [], [], [], []
        timePassed = 0
        torque = 0
        while (abs(self.getJointPos(joint) - targetPosition) > 0.0349):
            if('pos' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['pos'] = self.getJointPos(joint)
            if('vel' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['vel'] = 0
            self.jointsInfos[joint]['lastPos']   = self.jointsInfos[joint]['pos']
            self.jointsInfos[joint]['pos']       = self.getJointPos(joint)
            
            self.jointsInfos[joint]['lastVel']   = self.jointsInfos[joint]['vel']
            self.jointsInfos[joint]['vel']       = (self.jointsInfos[joint]['pos'] - self.jointsInfos[joint]['lastPos'])/ self.dt if self.dt >= 0.00001 else self.jointsInfos[joint]['lastVel']
            print(abs(self.getJointPos(joint) - targetPosition))
            pltTime.append(timePassed) 
            pltVelocity.append(self.jointsInfos[joint]['vel'])
            pltPosition.append(self.jointsInfos[joint]['pos'])
            pltTorque.append(torque)
            pltTarget.append(targetPosition)
            timePassed += self.dt
            torque = toy_tick(targetPosition, self.jointsInfos[joint]['pos'], targetVelocity, self.jointsInfos[joint]['vel'], integral=0)
            
            
        return pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity

    def move_with_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False, startJoint ="base_to_dummy"):
        """
        Move joints using inverse kinematics solver and using PD control.
        This method should update joint states using the torque output from the PD controller.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        targetStatess, jointNames, efDiffs, eflocations, pltTimes = self.inverseKinematics(endEffector=endEffector, targetPosition=targetPosition, 
                                                                orientation=orientation,
                                                                interpolationSteps=20,
                                                                threshold=threshold,
                                                                startJoint=startJoint,
                                                                debug = debug
                                                                )
                        
        #print("Done with kinematics")
            
        ## For debugging
        visualShift = eflocations[-1]
        inertiaShift = [0,0,0]

        meshScale=[0.1,0.1,0.1]
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1,0,0,1],radius= 0.015, visualFramePosition=visualShift, meshScale=meshScale)

        p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)

        for _ in range(maxIter):
        
            self.tick(targetStatess[-1], jointNames, speed)
            if(np.linalg.norm(self.getJointPosition(endEffector) -  eflocations[-1]) < threshold):
                #print("Reached treshold")
                break 
        #print("DONE")
        
        return pltTimes, eflocations
        

    def tick(self, targetStates, targetJoints, speed = 0.0):
        """Ticks one step of simulation using PD control."""
        # Iterate through all joints and update joint states using PD control.
        for i, joint in enumerate(targetJoints):
            # skip dummy joints (world to base joint)
            #print(joint)
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


            # set current and last positions and velocities
            # estimate velocity based on position diff and time
            if('pos' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['pos'] = self.getJointPos(joint)
            if('vel' not in self.jointsInfos[joint]):
                self.jointsInfos[joint]['vel'] = 0
            
            self.jointsInfos[joint]['lastPos']   = self.jointsInfos[joint]['pos']
            self.jointsInfos[joint]['pos']       = self.getJointPos(joint)
            
            self.jointsInfos[joint]['lastVel']   = self.jointsInfos[joint]['vel']        
            self.jointsInfos[joint]['vel'] =  (self.jointsInfos[joint]['pos'] - self.jointsInfos[joint]['lastPos']) /self.dt 
                
            torque = self.calculateTorque(  targetState, 
                                            self.jointsInfos[joint]['pos'],
                                            0, 
                                            self.jointsInfos[joint]['vel'],
                                            0, kp, ki, kd)  # TODO: fix me
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
    def cubic_interpolation(self, points, nTimes=100):
        """
        Given a set of control points, return the
        cubic spline defined by the control points,
        sampled nTimes along the curve.
        """
        time = np.linspace(0, nTimes, len(points))
        timerange = np.linspace(0, nTimes, nTimes)
        points = points.T

        cs_x = CubicSpline(time, points[0], bc_type = "natural")
        cs_y = CubicSpline(time, points[1], bc_type = "natural")
        cs_z = CubicSpline(time, points[2], bc_type = "natural")
        points = [[float(cs_x(i)), float(cs_y(i)), float(cs_z(i))] for i in timerange]

    
        return points

    # Task 3.1 Pushing
    def dockingToPosition(self):
        """A template function for you, you are free to use anything else"""        
        time.sleep(5)
        startPoint = self.getJointPosition("LARM_JOINT5") + np.array([0, 0, 0.85])         
        points = np.array([startPoint, [0.15, 0.1, 1],[0.12, -0.012, 0.96],[0.35, 0.01, 0.96],[0.55, 0.01, 0.96], [0.60, 0.01, 0.96]])
        # Simply use hardcoded points. We can also interpolate but its honestly not needed. 
        #points= self.cubic_interpolation(points, nTimes = 10)
        for p in points:
            self.move_with_PD("LARM_JOINT5", np.array(p) - np.array([0, 0, 0.85]), speed=0.01, orientation=[0,1,1], threshold=1e-3, maxIter=1000, debug=True, verbose=False, startJoint = "base_to_dummy")


    def move_with_PD_multiple(self, endEffectors, targetPositions, speed=0.01, orientations=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False, startJoint ="base_to_dummy"):
        """
        Move joints using inverse kinematics solver and using PD control.
        This method should update joint states using the torque output from the PD controller.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        
        
        targetStatess, jointNames, efDiffs, eflocations, pltTimes = self.inverseKinematics(endEffector=endEffectors[0], targetPosition=targetPositions[0], 
                                                                orientation=orientations[0],
                                                                interpolationSteps=20, #TODO: whats the  interpolation step here?
                                                                threshold=threshold,
                                                                startJoint="base_to_dummy",
                                                                debug=debug
                                                                )
        
        targetStatess_2, jointNames_2, efDiffs_2, eflocations_2, pltTimes_2 = self.inverseKinematics(endEffector=endEffectors[1], targetPosition=targetPositions[1], 
                                                                orientation=orientations[1],
                                                                interpolationSteps=20, #TODO: whats the  interpolation step here?
                                                                threshold=threshold,
                                                                startJoint="base_to_dummy",
                                                                debug=debug
                                                                )
        
        #print("Done with kinematics")

        final = np.concatenate([targetStatess_2[-1], targetStatess[-1][3:]])
        final[:3] = (targetStatess_2[-1][:3] + targetStatess[-1][:3]) / 2
        jointNames_2.extend(jointNames[3:])
        if debug: 
            visualShift = eflocations[-1]
            inertiaShift = [0,0,0]

            meshScale=[0.1,0.1,0.1]
            visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1,0,0,1],radius= 0.015, visualFramePosition=visualShift, meshScale=meshScale)

            p.createMultiBody(baseMass=0,baseInertialFramePosition=inertiaShift, baseVisualShapeIndex = visualShapeId, basePosition = [0,0,0.85], useMaximalCoordinates=False)

        for _ in range(maxIter):
        
            self.tick(final, jointNames_2, speed)
            if(np.linalg.norm(self.getJointPosition(endEffectors[0]) - eflocations[-1]) < threshold
               and 
               np.linalg.norm(self.getJointPosition(endEffectors[1] - eflocations_2[-1] < threshold))
            ):  
                #print("Break")
                break
                 
        #print("DONE")
        
        return pltTimes, eflocations
    # Task 3.2 Grasping & Docking
    def clamp(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005, threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

 ### END
