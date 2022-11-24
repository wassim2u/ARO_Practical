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
taskId = 1

try:
    if sys.argv[1] == 'nogui':
        gui = False
    else:
        gui = True
except:
    gui = True


### You may want to change the code from here
pybulletConfigs = {
    "simulation": bullet_simulation,
    "pybullet_extra_data": pybullet_data,
    "gui": gui,
    "panels": False,
    "realTime": False,
    "controlFrequency": 1000,
    "updateFrequency": 250,
    "gravity": -9.81,
    "gravityCompensation": .8,
    "floor": True,
    "cameraSettings": (1.07, 90.0, -52.8, (0.07, 0.01, 0.76))
}
robotConfigs = {
    "robotPath": core_path + "/nextagea_description/urdf/NextageaOpen.urdf",
    "robotPIDConfigs": core_path + "/PD_gains.yaml",
    "robotStartPos": [0, 0, 0.85],
    "robotStartOrientation": [0, 0, 0, 1],
    "fixedBase": True,
    "colored": False
}
sim = Simulation(pybulletConfigs, robotConfigs)

# This is an example target (angular) position for the joint LARM_JOINT2
task2_jointName = "LARM_JOINT2"
task2_targetPosition = np.deg2rad(-45)  # joint (angular) position in radians
task2_targetVelocity = 0.0  # joint (angular) velocity in radians per second
verbose = False
task2_figure_name = "task2_PD_response.png"
task2_savefig = True
### to here


pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = \
    sim.moveJoint(
        task2_jointName, task2_targetPosition, task2_targetVelocity, verbose)


# modify the code in below if needed
fig = plt.figure(figsize=(6, 8))

plt.subplot(311)
plt.plot(pltTime, pltPosition, color='blue', label="Joint Position")
plt.plot(pltTime, pltTarget, color='magenta', label="Target Position")
plt.ylabel("Theta rads")
leg = plt.legend(loc='upper right')
if verbose:
    print("Distance to target")
    print(pltPosition[-1] - pltTarget[-1])

plt.subplot(312)
plt.plot(pltTime, pltPosition, color='blue', label="Joint Position")
plt.plot(pltTime, pltVelocity, color='lightblue', label="Joint Velocity")
plt.ylabel("Velocity rads/s")
leg = plt.legend(loc='lower right')

plt.subplot(313)
plt.plot(pltTime, pltTorque, color='orange', label="Torque")
plt.xlabel("Time s")
plt.ylabel("Torque N")
leg = plt.legend(loc='lower right')

plt.suptitle("PD controller response for elbow", size=16)
plt.tight_layout()
plt.subplots_adjust(left=0.15)

if task2_savefig:
    fig.savefig(task2_figure_name)
plt.show()
