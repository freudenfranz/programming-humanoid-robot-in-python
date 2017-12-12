'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from angle_interpolation import AngleInterpolationAgent
from math import cos, sin

class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }

        self.joint_offsets = {  "HeadYaw":          [.0, .0, 126.50],   #from Torso
                                "HeadPitch":        [.0, .0, .0],       #from
                                #Left Shoulder
                                "LShoulderPitch":   [.0, 98.0, 100.0],  #from Torso
                                "LShoulderRoll":    [.0, .0, .0],       #from LShoulderPitch
                                "LElbowYaw":        [105.0, 15.0, 0.0], #from LShoulderRoll
                                "LElbowRoll":       [.0, .0, .0],       #from LElbowYaw
                                "LWristYaw":        [55.95, .0, .0],    #from LElbowRoll
                                #Left Leg
                                "LHipYawPitch":     [.0, 50.0, -85.0],  #from Torso
                                "LHipRoll":         [.0, .0, .0],       #from LHipYawPitch
                                "LHipPitch":        [.0, .0, .0],       #from LHipRoll
                                "LKneePitch":       [.0, .0, -100.0],   #from LHipPitch
                                "LAnklePitch":      [.0, .0, -102.9],   #from LKneePitch
                                "LAnkleRoll":       [.0, .0, .0],       #from LAnklePitch
                                #Right Shoulder
                                "RShoulderPitch":   [.0, -98.0, 100.0],  #from Torso
                                "RShoulderRoll":    [.0, .0, .0],       #from RShoulderPitch
                                "RElbowYaw":        [105.0, -15.0, 0.0], #from RShoulderRoll
                                "RElbowRoll":       [.0, .0, .0],       #from RElbowYaw
                                "RWristYaw":        [55.95, .0, .0],    #from RElbowRoll
                                #Right Leg
                                "RHipYawPitch":     [.0, -50.0, -85.0],  #from Torso
                                "RHipRoll":         [.0, .0, .0],       #from RHipYawPitch
                                "RHipPitch":        [.0, .0, .0],       #from RHipRoll
                                "RKneePitch":       [.0, .0, -100.0],   #from RHipPitch
                                "RAnklePitch":      [.0, .0, -102.9],   #from RKneePitch
                                "RAnkleRoll":       [.0, .0, .0],       #from RAnklePitch
                                }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        c = cos(joint_angle)
        s = sin(joint_angle)
        x_offset, y_offset, z_offset = self.joint_offsets[joint_name]
        last_row = [x_offset, y_offset, z_offset, 1.0]
        # YOUR CODE HERE
        if(joint_name == "LShoulderRoll"):
            joint_name = "LShoulderYaw"
        if(joint_name == "RShoulderRoll"):
            joint_name = "RShoulderYaw"
        if(joint_name == "LElbowYaw"):
            joint_name = "LElbowRoll"
        if(joint_name == "LElbowRoll"):
            joint_name = "LElbowYaw"

        if(joint_name.find('Roll')>0): #arround x-Axis -> Rx
            T=matrix([  [1.0, 0.0,  0.0, x_offset],
                        [0.0, c  , -s  , y_offset],
                        [0.0, s  ,  c  , z_offset],
                        [0.0, 0.0, 0.0 ,      1.0]])
        #    print "Rotate joint %s about x-axis (Roll)"%(joint_name)
        elif(joint_name.find('Pitch')>0): #arround y-Axis -> Ry
            T=matrix([  [ c  , 0.0, s  , x_offset],
                        [ 0.0, 1.0, 0.0, y_offset],
                        [-s  , 0.0, c  , z_offset],
                        [ 0.0, 0.0, 0.0,      1.0]])
        #    print "Rotate joint %s about y-axis (Pitch)"%(joint_name)
        elif(joint_name.find('Yaw')>0): #arround z -> Rz
            T=matrix([  [ c  , -s  , 0.0, x_offset],
                        [s  , c  , 0.0, y_offset],
                        [ 0.0, 0.0, 1.0, z_offset],
                        [ 0.0, 0.0, 0.0,      1.0]])
        #    print "Rotate joint %s about z-axis (Yaw) for %s"%(joint_name, str(joint_angle))
        else:
            print "WARNING: In Kinematics.local_trans() found neighter 'Pitch', 'Roll' nor 'Yaw' in joint_name"
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T= T.dot(Tl)
                #TODO mit letzter Matrix multiplizieren
                #TODO Winkel multiplizieren
                #print "Transform T=%"%str(T)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
