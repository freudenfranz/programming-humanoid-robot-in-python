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
import traceback


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        self.keyframes = None#([], [], [])
        self.interpolation_just_started = False
        self.interpolation_is_running = False

        #{name: [[tkey], [angle, 2x[interpType, dTime, dAngle]]]}
        self.keyframe_book = {}
        self.keyframe_times = {}
        self.keyframe_keys = {}
        self.t_right = {} # list of right sided keyframes of every segment/joint
        self.t_left = {} # list of left sided keyframes of every segment/joint

        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

    def set_keyframes(self, keyframes):
        '''Fills t_right and t_left for every joint, so we have two points
            we can start interpolating within. Also the rest of the keyframes are
            copied to better data-stuctures.
            Then in the actual interpolation t_left and t_right are shifted to
            the left if the time of t_right was passed.
            Since we have to come up with the very first t_left, we taken
            the angle that the joint is in now, and the handle1 from t_right.
            LLeg: t_left ..(now)->..t_right.....2.Keyframe......3.Keyframes..'''
        self.keyframes = keyframes
        index = 0
        try:
            for name in keyframes[0]: #take first time-stamp and key out of every joint
                #set t_right for joint 'name'
                self.keyframe_times[name] = self.keyframes[1][index]
                self.keyframe_keys[name] = self.keyframes[2][index]
                if len(self.keyframe_keys) != len(self.keyframe_times):
                    print "WARNING! Amount of timestamps and keyframes for joint %s doesn't korrespond"%name
                #self.keyframe_book[name] = [self.keyframes[1][index][0:], self.keyframes[2][index][0:]]
                #t1 = self.keyframe_book[name][0].pop(0)
                t1 = self.keyframe_times[name].pop(0)
                #k1 = self.keyframe_book[name][1].pop(0)
                k1 = self.keyframe_keys[name].pop(0)
                #t1 = self.keyframes[1][index][0]
                #k1 = self.keyframes[2][index][0]
                self.t_right[name] = [t1, k1]
                #
                #set t_left for joint 'name'
                a_0 = self.perception.joint[name] #angle at time 0 (=now)
                dt_0 = k1[1][1] #take same acceleration for first Timestampe then second ones has
                ipol_type = self.keyframes[2][index][0][1][0] #that is usually '3'
                k0 = [ a_0, [ipol_type, dt_0, 0.], [ipol_type, -dt_0, 0.] ]
                self.t_left[name] = [0, k0]
                #if self.keyframes[1][index][0]: #else we removed it allready to t_right, so don't copy again
                #if self.keyframes[1][index][1]: #there is only one time/angle  for this joint
                    #self.keyframe_book[name]    = [self.keyframes[1][index][1:], self.keyframes[2][index][1:]]
                if self.verbosity_level > 7:
                    if len(self.keyframe_times[name])==0:
                        print"\tangle_intp: found only one angle/timestamp for joint %s"%(name)
                index += 1 #next name(=joint)
            if self.verbosity_level > 7:
                for u in keyframes[0]:
                    print "\tset_keyframes:Set keypoints for joint %s at: \n\t\tleft=%s \n\t\tright=%s"%(u,self.t_left[u],self.t_right[u])
                print "\n\tset_keyframes: the left keypoints are:"
                for u in keyframes[0]:
                    print"\n\tset_keyframes: %s"%u
                    print"\tset_keyframes: timestamps: %s"%self.keyframe_times[name]
                    print"\tset_keyframes: keys: %s"%self.keyframe_keys[name]
            self.interpolation_just_started = True #Set Flag that Interpolation can start
        except:
            print "had some problem at set_keyframes"
            traceback.print_exc()


    def think(self, perception):
        target_joints = self.angle_interpolation(perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, perception):
        '''Iterpolates the angle between two key angles of a joint, but for a
            list of joints and respectiveley a list of keyangles in order to
            provide a smooth moovement.
            before you can use this function you have to define all joints which
            should be considered with 'set_keyframes()'
        '''
        target_joints = {}
        # YOUR CODE
        if self.interpolation_just_started:
            self.interpolation_start_time = perception.time
            print "Start-Time=%.4fs"%(self.interpolation_start_time/1000)
            self.interpolation_is_running = True
            self.interpolation_just_started = False

        if self.interpolation_is_running:#this is running constanty in the loop, so we won't do that if non necessary
            try:
                t_now = self.perception.time - self.interpolation_start_time
                to_remove = [] #if joints are not needed anymore, forget them..
                for name in self.keyframe_times:
                    if t_now > self.t_right[name][0]: #time passed? Then switch to next segment..
                        self.t_left[name] = self.t_right[name] #shift Keyframe of t_left
                        if len(self.keyframe_times[name]):     #there is annother frame in the queue
                            ti = self.keyframe_times[name].pop(0)
                            ki = self.keyframe_keys[name].pop(0)
                            self.t_right[name] = [ti, ki]      #shift Keyframe of t_right
                            if self.verbosity_level > 6:
                                print "\tangle_intp: For joint %s are after this %i keypoints left"%(name, len(self.keyframe_times[name]))
                                print "\tangle_intp: New angle is %s at time %s"%(ki[0], ti)
                        else: #no more frames in the queue, so joint ist done now
                            #del self.keyframe_times[name]
                            #del self.keyframe_keys[name]
                            to_remove.append(name)
                            if self.verbosity_level > 6:
                                print "\tangle_intp: joint %s is done after %.3fs"%(name, t_now)
                            if not len(self.keyframe_times): #everything done!!
                                self.interpolation_is_running = False
                                if self.verbosity_level > 6:
                                    print "\tangle_intp: All Joits are through"
                    #Interpolating!
                    target_joints[name] = self.calc_bezier(t_now, self.t_left[name], self.t_right[name])
                    #target_joints[name] = self.calc_linear(t_now, self.t_left[name], self.t_right[name])
                for name in to_remove:
                    self.keyframe_times.pop(name)
                    self.keyframe_keys.pop(name)
            except KeyError:
                if self.verbosity_level > 5:
                    print "\tangle_interpolation: had some Key issues!"
                traceback.print_exc()

        return target_joints

    def calc_linear(self, time_now, left, right):
        d_t = right[0] - left[0]
        d_angle = right[1][0] - left[1][0]
        return (d_angle/d_t) * (time_now - left[0]) + left[1][0]

    def calc_bezier(self, ti, left_handle, right_handle):
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
    agent.set_keyframes(leftBackToStand())  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = leftBackToStand()
    agent.run()
