from config import *
import log
from methods import isFeasible


#TODO load simulation with self collision




def testGenData():
	matrices, names = sim.forwardKinematics("RARM_JOINT5", sim.measureJointAngles())
	pos, angle = sim.extractPositionAndRotation(matrices[3])
	targets = np.array([[0.4, 0.1, 0.9]]) - pos
	sim.move_without_PD("RARM_JOINT5", targets[0], speed=0.9, orientation=np.array([-1, -1, 1]),
                                          threshold=10e-3, maxIter=1000, debug=True, verbose=False, startJoint="RARM_JOINT0")
	matrices, names = sim.forwardKinematics("LARM_JOINT5", sim.measureJointAngles())
	pos, angle = sim.extractPositionAndRotation(matrices[3])
	targets = np.array([[0.1, 0.75, 1]]) - pos
	sim.move_without_PD("LARM_JOINT5", targets[0], speed=0.9, orientation=np.array([0, -1, 0]),
                                          threshold=10e-3, maxIter=1000, debug=True, verbose=False, startJoint="LARM_JOINT0")
	matrices, names = sim.forwardKinematics("HEAD_JOINT0", sim.measureJointAngles())
	pos, angle = sim.extractPositionAndRotation(matrices[3])
	targets = np.array([[0.1, 0, 2]]) -pos - sim.baseOffset
	sim.move_without_PD("HEAD_JOINT1", targets[0], speed=0.9, orientation=np.array([1, 1, 1]),
                                          threshold=10e-3, maxIter=1000, debug=True, verbose=False, startJoint="HEAD_JOINT0")
	log.save_trajectory()


def loadTrajAndValidate():
	traj = log.trajectory()
	traj_ok = isFeasible(sim,traj)
	if traj_ok:
		print ("Trajectory is valid")
	else:
		print ("Trajectory is not valid")
	return traj_ok
	
	
if __name__ == "__main__":
	testGenData() 
	loadTrajAndValidate()

	traj = log.trajectory()
