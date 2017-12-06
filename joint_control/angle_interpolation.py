'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.interpolation_start_time = 0.00000

        #{name: [[tkey], [angle, 2x[interpType, dTime, dAngle]]]}
        self.keyframe_book = {}
        self.t_right = {} # list of right sided keyframes of every segment/joint
        self.t_left = {} # list of loft sided keyframest of every segment/joint

    def set_keyframes(self, keyframes, name):
        self.keyframes = keyframes
        print "Setting Keyframes to %s"%name
        index = 0
        for name in keyframes[0]: #take first time and key of every joint out
            t1 = self.keyframes[1][index][0]
            k1 = self.keyframes[2][index][0]
            self.t_right[name] = [t1, k1]

            dt_0 = k1[1][1]
            k0 = [0, #angle at time 0
            [self.keyframes[2][index][0][1][0], dt_0, 0.0], #interpType1, dT1, dangle1
            [self.keyframes[2][index][0][2][0], -dt_0, 0.0]  #interpType2, dT2, dangle2
            ]
            self.t_left[name] = [0, k0]
            if self.keyframes[1][index][0]: #else we removed it allready to t_right, so don't copy again
                self.keyframe_book[name] = [self.keyframes[1][index][1:], self.keyframes[2][index][1:]]

            index += 1
        '''print self.keyframe_book
        print ""
        print self.t_left
        print ""
        print self.t_right'''

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE
        #print "The time since inetrp.-start ist: %s"%(str(t_now-self.interpolation_start_time))
        if not self.interpolation_start_time:
            self.interpolation_start_time = perception.time
        t_now = self.perception.time - self.interpolation_start_time
        for name in keyframes[0]:
            if t_now > self.t_right[name][0]: #switch no next segment?
                self.t_left[name] = self.t_right[name]
                if self.keyframe_book: #all joints done with moving?
                    if name in self.keyframe_book: #maybe moovements for this joint were accomplised?
                        #print "accessing key: %s"%(name)
                        ti = self.keyframe_book[name][0].pop(0)
                        ki = self.keyframe_book[name][1].pop(0)
                        self.t_right[name] = [ti, ki]
                        #print self.keyframe_book[name]
                        if not self.keyframe_book[name][0]: #joint ist done now
                            del self.keyframe_book[name]
                            #print "joint %s is done"% (name)
                else:
                    self.interpolation_start_time = False
            if name in self.keyframe_book: #we have right handles
            #Interpolation!
                #target_joints[name] = self.calc_bezier(t_now, self.t_left[name], self.t_right[name], name)
                target_joints[name] = self.calc_linear(t_now, self.t_left[name], self.t_right[name])
        return target_joints

    def calc_linear(self, time_now, left, right):
        d_t = right[0] - left[0]
        d_angle = right[1][0] - left[1][0]
        return (d_angle/d_t) * (time_now - left[0]) + left[1][0]

    def calc_bezier(self, ti, left_handle, right_handle, name):
        t0 = left_handle[0]
        t1 = t0 + left_handle[1][2][1]
        t3 = right_handle[0]
        t2 = t3 + right_handle[1][1][1] #this dt is always neg
        #print"joint %s: t0= %f, t1=%f, t2=%f, t3=%f, ti =%f "%(name,t0,t1,t2,t3,ti)
        '''Koeffizienten der Polynoms abhaengig von der Zeit B(ti,_): K1*i^3+K2*i^2+K3*i+(K4-ti) wobei ti, da ja Nullstellen berechnet werden'''
        coeff_t = [-t0+3*t1-3*t2+t3, 3*t0-6*t1+3*t2, -3*t0+3*t1, t0-ti]
        null_points = np.roots(coeff_t)
        i = False
        for root in null_points:
            if np.isreal(root):
                if 0 <= root <=1: #lets say its the only one
                    #print"Yeagh!!! found root: %f"%np.real(root)
                    i = root

        #TODO see if roots make sense TODO remove name
        #print "roots of Polynoms: %s"%str(null_points)
        a0 = left_handle[1][0]
        a1 = left_handle[1][2][2] + a0
        a3 = right_handle[1][0]
        a2 = right_handle[1][1][2] + a3
        coeff_a= [-a0+3*a1-3*a2+a3, 3*a0-6*a1+3*a2, -3*a0+3*a1, a0]
        #print("a0=%f, a1=%f, a2=%f, a3=%f")%(a0, a1, a2, a3)
        return np.polyval(coeff_a, i)
if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.set_keyframes(leftBackToStand(), "leftBackToStand")  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = leftBackToStand()
    agent.run()
