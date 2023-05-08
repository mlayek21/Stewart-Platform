# Description: This file is used to test the stewart platform
import pybullet as p
import time
import pybullet_data
import os 
from inv_kinematics import inv_kinematics as ik
import numpy as np

class StewartPlatform:
    def __init__(self, path, joint_indices, actuator_indices, design_variable) -> None:
        self.path = path
        self.joint_indices = joint_indices
        self.actuator_indices = actuator_indices
        self.design_variable = design_variable
        self.prev_target = np.zeros(len(actuator_indices))

    def cls(self):
        os.system('cls')
    def set_env(self):
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        # physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        # Load the Stewart Platform
        self.robotId = p.loadURDF(self.path,cubeStartPos, cubeStartOrientation,
                                flags=p.URDF_USE_INERTIA_FROM_FILE,useFixedBase = 1)
                        
        # Set the camera position and orientation
        camera_target_position = [0, 0, 0]
        camera_distance = 1.5
        camera_yaw = 50
        camera_pitch = -35
        p.resetDebugVisualizerCamera(cameraDistance = camera_distance, 
                                    cameraYaw = camera_yaw,
                                    cameraPitch = camera_pitch,
                                    cameraTargetPosition = camera_target_position, 
                                    physicsClientId = physicsClient)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        self.n = p.getNumJoints(self.robotId)  # Get the number of joints in the robot
        self.Ind = {}                            # Create a dictionary to store the joint indices
        # Get the joint indices
        for i in range(self.n):
            joint_info = p.getJointInfo(self.robotId, i)
            joint_name = joint_info[1].decode("utf-8")
            self.Ind[joint_info[0]]=joint_name
            # p.changeVisualShape(self.robotId, i, rgbaColor=[1,0,0,1])
        return

    def set_constraints(self):
        # Create a fixed constraint for each pair of linked joints
        for parent_joint, child_joint in self.joint_indices:
            # Create the constraint
            constraint_id = p.createConstraint(self.robotId, parent_joint, self.robotId, child_joint, p.JOINT_FIXED, [0,0,0.1], [0,0,0], [0,0,0])
            # If you need to store the constraint ID for later use, you can add it to a list or dictionary here
            p.changeConstraint(constraint_id, maxForce=1e20)

        # Disable the motors for all joints
        for i in range(self.n):
            maxForce = 0
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2(self.robotId, i,
                                    controlMode=mode, force=maxForce)
        return
        
    def init_stewart(self,flag):
        # Design variables
        r_P,r_B, gama_P,gama_B = self.design_variable
        self.clf = ik(r_P,r_B,gama_P,gama_B)
        # compute the leg length for the initial position
        translation = np.array([0, 0, 0])       # translation in meters
        rotation = np.array([0, 0, 0])         # rotation in degrees
        self.l  = self.clf.solve(translation, rotation)
        # print("leg length",self.l)
        if flag:
            translation = np.array([0, 0, 0.09])
            leg_1  = self.clf.solve(translation, rotation)
            # print("leg length",self.l,leg_1)
            self.linear_actuator(leg_1-self.l, 1)
            time.sleep(1.)
        return 
    
    # Motor driver function
    def linear_actuator(self, linear_distance, actuation_duration):
        # Calculate the step size and the PWM parameters
        frequency = 50                                   # 50 Hz
        self.max_force = 250                                # 250 N linear actuation force
        steps = int(actuation_duration * frequency)  # pwm steps
        pwm_period = 1.0 / frequency                  # pwm period
        duty_cycle = 0.5                            # 50% duty cycle
        pwm_high_time = pwm_period * duty_cycle 
        actuation_step = np.array([l / steps for l in linear_distance]) # actuation step
        # Actuate the linear actuators
        for i in range(steps):
            # Read the pwm signal
            pwm_signal = i * pwm_period % pwm_period < pwm_high_time
            # pwm signal is high then actuate the linear actuator
            if pwm_signal:
                # print("PWM signal is high")
                # Calculate the target position
                target_position = actuation_step * i + self.prev_target

            else:
                # print("PWM signal is low")
                # Hold the position
                target_position = np.array([p.getJointState(self.robotId, self.actuator_indices[j])[0]
                                            for j in range(len(self.actuator_indices))])
            
            # Actuate the linear actuator
            p.setJointMotorControlArray(self.robotId, self.actuator_indices, 
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPositions = target_position, 
                                        forces=self.max_force*np.ones(len(self.actuator_indices)))
            time.sleep(1./(frequency))
            p.stepSimulation()
            
        self.prev_target = np.array(actuation_step) * i+ self.prev_target
        # print("actuation step",self.prev_target)
        return 
    def reset_position(self):
        time.sleep(1)
        # # Reset the position of the robot
        self.linear_actuator(-self.prev_target, 1)
        p.setJointMotorControlArray(self.robotId, self.actuator_indices, 
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPositions = np.zeros(len(self.actuator_indices)), 
                                        forces=self.max_force*np.ones(len(self.actuator_indices)))
        return
       
    def start_simmulation(self, data, simulation=False, flag = True):
        self.set_env()         # set the environment
        self.set_constraints()  # set the constraints
        if simulation:
            logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "simulation.mp4")
        self.init_stewart(flag)  # initialize the stewart platform
        for i in data:
            trans,rot,t = i
            l = self.clf.solve(trans, rot) # compute the leg length
            dl = l-self.l                    # compute the leg actuation distance
            self.linear_actuator(dl, t)         # actuate the linear actuator
                       # reset the position
        self.reset_position()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        for i in range (100):
            p.stepSimulation()
            time.sleep(1./240.)
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        
        # print(cubePos,cubeOrn)
        # Stop recording the simulation
        if simulation:
            p.stopStateLogging(logging_id)
        p.disconnect()
        return
    
    def fit(self, datas, flag, simulation=False):
        self.set_env()         # set the environment
        self.set_constraints()  # set the constraints
        
        if simulation:
            logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "simulation.mp4")
        
        for i,data in enumerate(datas):
            self.init_stewart(flag[i])  # initialize the stewart platform
            # print(data)
            for i in data:
                trans,rot,t = i
                l = self.clf.solve(trans, rot) # compute the leg length
                dl = l-self.l                    # compute the leg actuation distance
                self.linear_actuator(dl, t)         # actuate the linear actuator
            
            # reset the position
            self.reset_position()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        for i in range (100):
            p.stepSimulation()
            time.sleep(1./240.)
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        
        # print(cubePos,cubeOrn)
        # Stop recording the simulation
        if simulation:
            p.stopStateLogging(logging_id)
        p.disconnect()
        return