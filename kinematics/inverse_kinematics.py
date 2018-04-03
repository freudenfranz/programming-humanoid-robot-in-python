'''In this exercise you need to implement inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from numpy import matrix, set_printoptions, get_printoptions
from math import pi, acos,cos, sin, atan2, pow, sqrt, atan2, asin
import numpy
from time import sleep
import traceback


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def rotate_about_x(self, matrix4x4, angle):
        rot_x=matrix([  [1.0,       0.0,          0.0, 0.0],
                        [0.0, cos(angle), -sin(angle), 0.0],
                        [0.0, sin(angle),  cos(angle), 0.0],
                        [0.0,       0.0,          0.0, 1.0]])
        return matrix4x4.dot(rot_x)

    def rotate_about_y(self, matrix4x4, angle):
        rot_y=matrix([  [cos(angle),        0.0,  sin(angle), 0.0],
                        [       0.0,        1.0,         0.0, 0.0],
                        [-sin(angle),       0.0,  cos(angle), 0.0],
                        [       0.0,        0.0,         0.0, 1.0]])
        return matrix4x4.dot(rot_y)

    def rotate_about_z(self, matrix4x4, angle):
        rot_z=matrix([  [cos(angle), -sin(angle), 0.0, 0.0],
                        [sin(angle),  cos(angle), 0.0, 0.0],
                        [       0.0,         0.0, 1.0, 0.0],
                        [       0.0,         0.0, 0.0, 1.0]])
        return matrix4x4.dot(rot_z)

    def translate_about_z(self, matrix4x4, distance):
        tra_z=matrix([  [ 1.0, 0.0, 0.0,      0.0],
                        [ 0.0, 1.0, 0.0,      0.0],
                        [ 0.0, 0.0, 1.0, distance],
                        [ 0.0, 0.0, 0.0,      1.0]])
        return matrix4x4.dot(tra_z)
    def rad2deg(self, radians):
        degrees = 180 * radians / pi
        return degrees

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        if   effector_name == 'LLeg':
            joint_angles = self.inverse_LLeg(joint_angles, transform)
        elif effector_name == 'RLeg':
            print "Not jet implemented"
        elif effector_name == 'LArm':
            print "Not jet implemented"
        elif effector_name == 'RArm':
            print "Not jet implemented"
        elif effector_name == 'Head':
            print "Not jet implemented"
        else:
            if selt.verbosity_level>2:
                print "\tinv_kin: No effector with name %s found"%effector_name
        return joint_angles

    def inverse_LLeg(self, joint_angles, transform):
        if self.verbosity_level > 3:
            print '\tinv_kin: Calculating inverse kinematics for left leg'
            joint_angles = {}
            '''"LHipYawPitch": -0.9 }
            ,"LHipYawPitch": 0.707", "LHipPitch": 0.707,
            "LHipRoll": 0.707, "LKneePitch": 0.707, "LAnklePitch": 0.707, "LAnkleRoll": 0.707}'''
            if self.verbosity_level > 3:
                print "\tinv_kin: added joints to joint_angles:\n%s"%joint_angles
        return joint_angles

    def inverse_RLeg(self, joint_angles, transform):
        return True
    def inverse_LArm(self, joint_angles, transform):
        return True
    def inverse_RArm(self, joint_angles, transform):
        return True
    def inverse_Head(self, joint_angles, transform):
        return True

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results

            * Keyframe data format:
                keyframe := (names, times, keys)
                names := [str, ...]  # list of joint names
                times := [[float, float, ...], [float, float, ...], ...]
                # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
                keys := [[float, [int, float, float], [int, float, float]], ...]
                # keys is a list of angles in radians (won't work with set_keyframes)
                # or an array of arrays each containing [float angle, Handle1, Handle2],
                # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
                # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
                # preceding the point, the second describes the curve following the point.
        '''
        # YOUR CODE HERE
        #TODO give the possibillity to change the executing time..
        duration = 3 #execution time for the movement to an new angle
        joint_angles = self.inverse_kinematics(effector_name, transform)

        #generate keyframes
        keyframes = [[],[],[]]
        names = list(joint_angles.keys())
        times = []
        keys = []
        for n in names:
            times.append([0., duration])
            #TODO check if Handles work that way..
            keys.append([[self.perception.joint.get(n),[3,-0.33,0.],[3,0.33,0.]],[joint_angles[n], [3, -0.33,0.],[3, 0.33, 0.]]])
        keyframes = [names, times, keys]
        keyframes[0].append("LHipPitch")
        keyframes[1].append([duration])
        keyframes[2].append([
            [-0.8, [3, -0.26667, 0.0], [3, 0.25333, 0.0]]
        ])
        self.set_keyframes(keyframes)
        if self.verbosity_level > 4:
            print "\tinv_kin: generated keyframes: \n%s"%str(self.keyframes)
        #self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    #agent.run()
