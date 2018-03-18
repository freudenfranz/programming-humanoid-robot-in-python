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
from numpy import matrix
from math import pi, acos,cos, sin, atan2, pow, sqrt, atan2, asin
from time import sleep


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def rotate_about_x(self, angle):
        rot_x=matrix([  [1.0,       0.0,        0.0, 0.0],
        [0.0, cos(angle), -sin(angle), 0.0],
        [0.0, sin(angle),  cos(angle), 0.0],
        [0.0,       0.0,        0.0, 1.0]])
        return rot_x
    def rotate_about_y(self, angle):
        rot_y=matrix([  [cos(angle),        0.0,  sin(angle), 0.0],
        [       0.0,        1.0,         0.0, 0.0],
        [-sin(angle),       0.0,  cos(angle), 0.0],
        [       0.0,        0.0,         0.0, 1.0]])
        return rot_y
    def rotate_about_z(self, angle):
        rot_z=matrix([  [cos(angle), -sin(angle), 0.0, 0.0],
        [sin(angle),  cos(angle), 0.0, 0.0],
        [       0.0,         0.0, 1.0, 0.0],
        [       0.0,         0.0, 0.0, 1.0]])
        return rot_z
    def translate_about_z(self, distance):
        tra_z=matrix([  [ 1.0, 0.0,      0.0, 0.0],
        [ 0.0, 1.0,      0.0, 0.0],
        [ 0.0, 0.0,      1.0, 0.0],
        [ 0.0, 0.0, distance, 1.0]])
        return tra_z
    def rad2deg(self, radians):
        degrees = 180 * radians / pi
        return degrees

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        verbose = True

        self.bodypart_sizes= {  "Hip_offset_Z":     85.0,
        "Hip_offset_Y":     50.0,
        "Thigh_lenght":     100.0,#oberschenkel
        "Tibia_length":     102.9,#schienbein
        "Foot_height":      45.19}

        if(effector_name.find('Leg')>0):
            if(effector_name =='LLeg'):
                left = True
                if verbose:
                    print 'Calculating inverse kinematics for left leg'
            elif(effector_name == 'RLeg'):
                left = False
                if verbose:
                    print 'Calculating inverse kinematics for right leg'
            else:
                print "WARNING effector was neighter RLeg nor LLeg"

            #TODO: was ist fuer links, was fuer rechts?? Winkel unten anpassen!!
            trans_y = identity(4)
            if left:
                trans_y[3,1]=self.bodypart_sizes["Hip_offset_Y"]
            else:
                trans_y[3,1]=-self.bodypart_sizes["Hip_offset_Y"]
            if verbose:
                print "trans_y = \n%s "%str(trans_y)

        # TODO Winkel gleich einrechnen?    foot2hip = trans_y.dot(transform)
            rot_x=self.rotate_about_x(pi/4)
            if verbose:
                print "foot2hip = \n%s "%str(foot2hip)
            foot2hip_orthogonal = foot2hip.dot(rot_x)
            if verbose:
                print "foot2hip_orthogonal = \n%s "%str(foot2hip_orthogonal)
            hip_orthogonal2foot = foot2hip_orthogonal#.inverse
            #TODO richtig invertieren
            if verbose:
                print "hip_orthogonal2foot = \n%s "%str(hip_orthogonal2foot)
            x_ortho=hip_orthogonal2foot[3,1]
            y_ortho=hip_orthogonal2foot[3,2]
            z_ortho=hip_orthogonal2foot[3,3]
            l_trans = sqrt(
                            pow(x_ortho,2)+
                            pow(y_ortho,2)+
                            pow(z_ortho,2)
                            )
            if verbose:
                print "l_trans = %s "%str(l_trans)
                #TODO: Laenge kann nicht stimmen
            knee_angle = pi - acos(
                                     (
                                        pow(self.bodypart_sizes["Thigh_lenght"] , 2)+
                                        pow(self.bodypart_sizes["Tibia_length"], 2)-
                                        pow(                      l_trans      , 2)
                                     )
                                        /
                                     (
                                        2 *
                                           self.bodypart_sizes["Thigh_lenght"] *
                                           self.bodypart_sizes["Tibia_length"]
                                     )
                                  )
            if verbose:
                print "knee_angle (delta_knee) = %s rad"%str(knee_angle)
                print "knee_angle (delta_knee) = %s * pi"%str(knee_angle/pi)
                print "knee_angle (delta_knee) = %s * deg"%str(self.rad2deg(knee_angle))

            foot_pitch_angle_1=acos(
                                     (
                                        -pow(self.bodypart_sizes["Thigh_lenght"], 2)+
                                        pow(self.bodypart_sizes["Tibia_length"], 2)+
                                        pow(                            l_trans, 2)
                                     )
                                        /
                                     (2*self.bodypart_sizes["Thigh_lenght"]*l_trans)
                                  )
            if verbose:
                print "foot_pitch_angle_1 = %s rad"%str(foot_pitch_angle_1)
                print "foot_pitch_angle_1 = %s * pi"%str(foot_pitch_angle_1/pi)
                print "foot_pitch_angle_1 = %s * deg"%str(self.rad2deg(foot_pitch_angle_1))

            foot_pitch_angle_2 = atan2(x_ortho, sqrt(pow(y_ortho,2)+pow(z_ortho,2)))
            if verbose:
                print "foot_pitch_angle_2 = %s rad"%str(foot_pitch_angle_2)
                print "foot_pitch_angle_2 = %s * pi"%str(foot_pitch_angle_2/pi)
                print "foot_pitch_angle_2 = %s * deg"%str(self.rad2deg(foot_pitch_angle_2))

            foot_pitch_angle = foot_pitch_angle_1 + foot_pitch_angle_2
            if verbose:
                print "foot_pitch_angle = %s rad"%str(foot_pitch_angle)
                print "foot_pitch_angle = %s * pi"%str(foot_pitch_angle/pi)
                print "foot_pitch_angle = %s * deg"%str(self.rad2deg(foot_pitch_angle))

            foot_roll_angle = atan2(y_ortho, z_ortho)
            if verbose:
                print "foot_roll_angle = %s rad"%str(foot_roll_angle)
                print "foot_roll_angle = %s * pi"%str(foot_roll_angle/pi)
                print "foot_roll_angle = %s * deg"%str(self.rad2deg(foot_roll_angle))

            thigh2foot =    self.rotate_about_x(foot_roll_angle).dot(
                            self.rotate_about_y(foot_pitch_angle).dot(
                            self.translate_about_z(self.bodypart_sizes["Tibia_length"]).dot(
                            self.rotate_about_y(knee_angle).dot(
                            self.translate_about_z(self.bodypart_sizes["Thigh_lenght"])
                            ))))
            if verbose:
                print "thigh2foot = \n%s "%str(thigh2foot)

            inv_thigh2foot = identity(4)#TODO richtig invertieren
            hip_orthogonal2thigh = inv_thigh2foot.dot(hip_orthogonal2foot)
            if verbose:
                print "hip_orthogonal2thigh = \n%s "%str(hip_orthogonal2thigh)

            hip_Yaw_angle = atan2(-hip_orthogonal2thigh[0,1], hip_orthogonal2thigh[1,1])
            if verbose:
                print "hip_Yaw_angle = %s rad"%str(hip_Yaw_angle)
                print "hip_Yaw_angle = %s * pi"%str(hip_Yaw_angle/pi)
                print "hip_Yaw_angle = %s * deg"%str(self.rad2deg(hip_Yaw_angle))

            hip_Pitch_angle = atan2(-hip_orthogonal2thigh[2,0], hip_orthogonal2thigh[2,2])
            if verbose:
                print "hip_Pitch_angle = %s rad"%str(hip_Pitch_angle)
                print "hip_Pitch_angle = %s * pi"%str(hip_Pitch_angle/pi)
                print "hip_Pitch_angle = %s * deg"%str(self.rad2deg(hip_Pitch_angle))

            hip_Roll_angle = asin(hip_orthogonal2thigh[2,1]) - pi/4
            if verbose:
                print "hip_Roll_angle = %s rad"%str(hip_Roll_angle)
                print "hip_Roll_angle = %s * pi"%str(hip_Roll_angle/pi)
                print "hip_Roll_angle = %s * deg"%str(self.rad2deg(hip_Roll_angle))

            hip_Pitch_angle = 0 #TODO
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
