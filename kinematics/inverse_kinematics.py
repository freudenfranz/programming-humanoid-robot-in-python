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

    def inverse_kinematics(self, effector_name, Foot2Torso):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        target_x = Foot2Torso[0,3]
        target_y = Foot2Torso[1,3]
        target_z = Foot2Torso[2,3]
        if self.verbosity_level > 4:
            print "\ninv_kin: Try to moove to %.2f %.2f %.2f seen from the Torso"%(target_x, target_y, target_z)

        self.bodypart_sizes= {
                                "Hip_offset_Z":     85.0,
                                "Hip_offset_Y":     50.0,
                                "Thigh_lenght":     100.0,#oberschenkel
                                "Tibia_length":     102.9,#schienbein
                                "Foot_height":      45.19}

        if(effector_name.find('Leg')>0):
            #----- Select calculationen depending on which limb is given
            if(effector_name =='LLeg'):
                original = get_printoptions()
                set_printoptions(precision=2)
                set_printoptions(suppress=True)
                if self.verbosity_level > 3:
                    print '\tinv_kin: Calculating inverse kinematics for left leg'

                TransY = identity(4)
                TransY[1,3]= -self.bodypart_sizes["Hip_offset_Y"]
                TransY[2,3]= self.bodypart_sizes["Hip_offset_Z"]

                Foot2Hip = TransY.dot(Foot2Torso)
                if self.verbosity_level > 4:
                    print"\ninv_kin: Foot2Hip=\n%s"%Foot2Hip

                Foot2HipOrthogonal = self.rotate_about_x(Foot2Hip, -pi/4)
                if self.verbosity_level > 4:
                    print"\ninv_kin: Foot2HipOrthogonal=\n%s"%Foot2HipOrthogonal

                HipOrthogonal2Foot = Foot2HipOrthogonal.I

                if self.verbosity_level > 4:
                    print "\ninv_kin: HipOrthogonal2Foot =\n%s"%HipOrthogonal2Foot
                #Translation = HipOrthogonal2Foot.dot([[target_x], [target_y], [target_z], [0]])

                #TODO kann sein, dass letzte stell im vektor ne 1 ist?
                try:
                    #Calculate Joints determined by the translation_vector HipO->Foot
                    tx = HipOrthogonal2Foot[0, 3]#Translation[0]
                    ty = HipOrthogonal2Foot[1, 3]#Translation[1]
                    tz = HipOrthogonal2Foot[2, 3]#Translation[2]
                    t = sqrt(tx*tx+ty*ty+tz*tz)
                    if self.verbosity_level > 4:
                        print "\tinv_kin: HipO2Foot=%.2f with x,y,z=(%.2f, %.2f, %.2f)"%(t, tx,ty,tz)

                    u = self.bodypart_sizes["Thigh_lenght"]
                    l = self.bodypart_sizes["Tibia_length"]
                    if self.verbosity_level > 5:
                        print "\tinv_kin: lower_limb= %s, upper_limb=%s"%(l,u)

                    #KNEE
                    angle_knee = pi-acos((u*u+l*l-t*t)/(2*u*l))
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LKneePitch is: %.3frad = %.2fgrad"%(angle_knee, angle_knee * 180 / pi)
                    #FOOT-PITCH
                    angle_foot_pitch1 = acos((l*l+t*t-u*u)/(2*l*t))
                    if self.verbosity_level > 4:
                        print "\tinv_kin: angle of foot_pitch1 is: %.3frad = %.2fgrad"%(angle_foot_pitch1, angle_foot_pitch1 * 180 / pi)

                    angle_foot_pitch2 = atan2(tx, sqrt(ty*ty+tz*tz))
                    if self.verbosity_level > 4:
                        #print "\ninv_kin: sqrt(y*y+z*z)=%.3fmm x= %.2fmm"%(sqrt(ty*ty+tz*tz),tx)
                        print "\tinv_kin: angle of foot_pitch2 is: %.3frad = %.2fgrad"%(angle_foot_pitch2, angle_foot_pitch2 * 180 / pi)

                    angle_foot_pitch  = angle_foot_pitch1 + angle_foot_pitch2
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LAnklePitch is: %.3frad = %.2fgrad"%(angle_foot_pitch, angle_foot_pitch * 180 / pi)
                    #ANKLE
                    angle_foot_roll = atan2(ty,tz)
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LAnkleRoll is: %.3frad = %.2fgrad"%(angle_foot_roll, angle_foot_roll * 180 / pi)
                        #print "\ninv_kin: ty=%.2fmm, tz=%.2fmm"%(ty,tz)


                    #Calculate remaining jonts of Hip
                    StartMatrix = identity(4) #TODO Mit welcher Matrix wird wirklich angefangen?
                    Foot_Roll  = self.rotate_about_x(StartMatrix,   angle_foot_roll)
                    Foot_Pitch = self.rotate_about_y(Foot_Roll,     angle_foot_pitch)
                    Lower_Leg  = self.translate_about_z(Foot_Pitch, l)
                    Knee       = self.rotate_about_y(Lower_Leg,     angle_knee)
                    Thigh2Foot  = self.translate_about_z(Knee,       u)
                    if self.verbosity_level > 4:
                        print "\tThigh2Foot is: \n%s"%(str(Thigh2Foot))
                    InvThigh2Foot = Thigh2Foot.I
                    if self.verbosity_level > 4:
                        print "\tThigh2Foot is: \n%s"%(str(InvThigh2Foot))
                    HipOrthogonal2Thigh = InvThigh2Foot.dot(HipOrthogonal2Foot)
                    if self.verbosity_level > 4:
                        print "\tHipOrthogonal2Thigh is: \n%s"%(str(HipOrthogonal2Thigh))

                    angle_hip_roll = asin(HipOrthogonal2Thigh[2,1]) - pi/4
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LHipRoll is: %.3frad = %.2fgrad"%(angle_hip_roll, angle_hip_roll * 180 / pi)

                    angle_hip_yaw = atan2(-HipOrthogonal2Thigh[0,1], HipOrthogonal2Thigh[1,1])
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LHipYaw is: %.3frad = %.2fgrad"%(angle_hip_yaw, angle_hip_yaw * 180 / pi)

                    angle_hip_pitch = atan2(-HipOrthogonal2Thigh[2,0], HipOrthogonal2Thigh[2,2])
                    if self.verbosity_level > 3:
                        print "\tinv_kin: angle of LHipPitch is: %.3frad = %.2fgrad"%(angle_hip_pitch, angle_hip_pitch * 180 / pi)


                    set_printoptions(**original)
                except:
                    traceback.print_exc()





        elif(effector_name == 'RLeg'):
            left = False
            if verbose:
                print 'Calculating inverse kinematics for right leg'
            trans_y[3,1]=-self.bodypart_sizes["Hip_offset_Y"]
        else:
            print "WARNING effector was neighter RLeg nor LLeg"
            return

            #TODO: was ist fuer links, was fuer rechts?? Winkel unten anpassen!!
            if self.verbosity_levelget > 3:
                print "trans_y = \n%s "%str(trans_y)

        joint_angles = []
        return joint_angles

    def pack_keyframes():

        return 1


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.inverse_kinematics(effector_name, transform);#TODO remove
        # YOUR CODE HERE
        self.set_keyframes(([], [], []))  # the result joint angles have to fill in
        #self.keyframes = ([],[],[])

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
