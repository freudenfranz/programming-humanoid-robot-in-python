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
from math import pi, acos,cos, sin, atan2, pow, sqrt, atan, asin
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
            if self.verbosity_level>2:
                print "\tinv_kin: No effector with name %s found"%effector_name
        return joint_angles

    def inverse_LLeg(self, joint_angles, transform):
        try:
            original = get_printoptions()
            set_printoptions(precision=3)
            set_printoptions(suppress=True)
            if self.verbosity_level > 3:
                print '\tinv_kin: Calculating inverse kinematics for left leg'
                joint_angles = {}
                '''"LHipYawPitch": -0.9 }
                ,"LHipYawPitch": 0.707", "LHipPitch": 0.707,
                "LHipRoll": 0.707, "LKneePitch": 0.707, "LAnklePitch": 0.707, "LAnkleRoll": 0.707}'''
            T = matrix(transform) #Chain from Torso to LFoot
            if self.verbosity_level > 3:
                print '\tinv_kin: T = \n%s'%str(T)
            A_Base = identity(4) #Left-ended translation-matrix of chain (TorsoToHip)
            A_Base[1,3]= self.bodypart_sizes["Hip_offset_Y"]
            A_Base[2,3]= -self.bodypart_sizes["Hip_offset_Z"]
            A_Base = A_Base.I
            if self.verbosity_level > 3:
                print '\tinv_kin: A_Base = \n%s'%str(A_Base)

            A_End = identity(4) #Right-ended translation-matrix of chain (AnkleToFoot)
            A_End[2,3] = -self.bodypart_sizes["Foot_height"]
            A_End = A_End.I
            if self.verbosity_level > 3:
                print '\tinv_kin: A_End = \n%s'%str(A_End)

            T_hut = A_Base.dot(T.dot(A_End))#Hip2Ankle (=202.9)
            ident = identity(4)
            T_tilde = self.rotate_about_x(ident, pi/4)#Hip2AnkleOrtho (=143,47)
            T_tilde = T_tilde.dot(T_hut)
            T_dash  = T_tilde.I #AnkleToHipOrthogonal
            if self.verbosity_level > 3:
                print '\tinv_kin: Hip2Ankle=T^ = \n%s'%str(T_hut)
                print '\tinv_kin: T~ = \n%s'%str(T_tilde)
                print '\tinv_kin: AnkleToHipOrthogonal=T\' = \n%s'%str(T_dash)

            t = T_dash  [:-1, -1]
            t = numpy.asarray(t)
            t = numpy.linalg.norm(t)

            u = self.bodypart_sizes["Thigh_lenght"]
            l = self.bodypart_sizes["Tibia_length"]
            argument=(u*u+l*l-t*t)/(2*u*l)
            if self.verbosity_level > 3:
                print 't=\n%s'%str(t)
                print('argument=%.3f')%(argument)

            #LKneePitch
            theta_4 = pi-acos((u*u+l*l-t*t)/(2*u*l))
            #TODO theta_4_n = -theta_4_p
            if self.verbosity_level > 3:
                print "\tinv_kin: angle of LKneePitch is: +-%.3frad = %.2fgrad"%(theta_4, theta_4 * 180 / pi)

            #LAnkleRoll
            theta_6 = atan(T_dash[0, 2] / T_dash[1, 2])
            if self.verbosity_level > 3:
                print "\tinv_kin: angle of LAnkleRoll is: +-%.3frad = %.2fgrad"%(theta_6, theta_6 * 180 / pi)
                print "WARNUNG. NUR Wenn bbla ungleich 0!!"

            T_5_6 = self.local_trans("LAnkleRoll", theta_6)
            if self.verbosity_level > 3:
                print '\tinv_kin: T_5->6 = \n%s'%str(T_5_6)


            T_rot_removed = self.rotate_about_y(self.rotate_about_z(T_5_6, pi), (-pi/2))
            if self.verbosity_level > 3:
                print '\tinv_kin: T~\' = \n%s'%str(T_rot_removed)

            T_tilde_dash = T_tilde.dot(T_rot_removed.I)
            if self.verbosity_level > 3:
                print '\tinv_kin: T~\' = \n%s'%str(T_tilde_dash)
                
            T_dash_2 = ((T_dash.I).dot(T_rot_removed.I)).I
            #T_dash_2 = T_tilde_dash.I
            if self.verbosity_level >   3:
                print '\tinv_kin: T\'\' = \n%s'%str(T_dash_2)

            #LAnklePitch
            #u = self.bodypart_sizes["Thigh_lenght"]
            #l = self.bodypart_sizes["Tibia_length"]
            #l1 = u = ThighLength and l2 =l= TibiaLength.
            quotient =  T_dash_2[1, 3] * (l+u*cos(theta_4))+ u * T_dash_2[0, 3] * sin(theta_4)
            dividend = l**2*sin(theta_4)**2+(l+u*cos(theta_4))**2
            if self.verbosity_level > 3:
                print 'divident = %s'%quotient
                print 'divisor = %s'%dividend
            theta_5 = asin(-(quotient/dividend))
            if self.verbosity_level > 3:
                print "\tinv_kin: angle of LAnklePitch is: +-%.3frad = %.2fgrad"%(theta_5, theta_5 * 180 / pi)

            T_3_4 = self.local_trans("LKneePitch", theta_4)
            T_4_5 = self.local_trans("LAnklePitch", theta_5)
            T_dash_3 = T_tilde_dash.dot((T_3_4.dot(T_4_5)).T)
            if self.verbosity_level > 3:
                print '\tinv_kin: T_3->4 = \n%s'%str(T_3_4)
                print '\tinv_kin: T_4->5 = \n%s'%str(T_4_5)
                print '\tinv_kin: T\'\'\' = \n%s'%str(T_dash_3)
                print '\tinv_kin: T\'\'\' translation block must be zero'

            theta_2_hut = acos(T_dash_3[1, 2])
            theta_2 = theta_2_hut - (pi/4)
            theta_3 = asin((T_dash_3[1,1])/(sin(theta_2+pi/4)))
            theta_1_hut = asin((T_dash_3[0,3])/(sin(theta_2+pi/4)))
            theta_1 = theta_1_hut + (pi/2)
            if self.verbosity_level > 3:
                print "\tinv_kin: Theta_2_hut is: +-%.3frad = %.2fgrad"%(theta_2_hut, theta_2_hut * 180 / pi)
                print "\tinv_kin: angle of Theta_2 is: +-%.3frad = %.2fgrad"%(theta_2, theta_2 * 180 / pi)
                print "\tinv_kin: angle of Theta_3 is: +-%.3frad = %.2fgrad"%(theta_3, theta_3 * 180 / pi)
                print "\tinv_kin: angle of Theta_1_hut is: +-%.3frad = %.2fgrad"%(theta_1_hut, theta_1_hut * 180 / pi)
                print "\tinv_kin: angle of Theta_1 is: +-%.3frad = %.2fgrad"%(theta_1, theta_1 * 180 / pi)



            #TODO falschen wert Filtern
            joint_angles['LHipYawPitch']= theta_1
            joint_angles['LHipRoll']    = theta_2
            joint_angles['LHipPitch']   = theta_3
            joint_angles['LKneePitch']  = theta_4
            joint_angles['LAnklePitch'] = theta_5
            joint_angles['LAnkleRoll']  = theta_6
            if self.verbosity_level > 3:
                print "\tinv_kin: added joints to joint_angles:\n%s"%joint_angles

            set_printoptions(**original) #reset Matrix formatting
            return joint_angles
        except:
            traceback.print_exc()

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
        print joint_angles

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
        # TEST-VALUEs
        #keyframes[0].append("LHipPitch")
        #keyframes[1].append([duration])
        #keyframes[2].append([
        #    [-0.8, [3, -0.26667, 0.0], [3, 0.25333, 0.0]]
        #])

        if self.verbosity_level > 4:
            print "\tinv_kin: generated keyframes: \n%s"%str(keyframes)
        self.set_keyframes(keyframes)
        #self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, -1] = 50.
    T[2, -1] = -333.09
    agent.set_transforms('LLeg', T)
    #agent.run()
