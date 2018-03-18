'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib
import sys
import os
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from threading import Thread
from numpy.matlib import identity

class PostHandler(object):
    '''the post handler wraps functions to be executed in parallel
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE
        client_Thread = Thread(target=self.proxy.execute_keyframes, args=(keyframes))
        return client_Thread

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        #taskID = Thread(set_tranform, (effector_name, transform))
        return taskID

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    def __init__(self, server_url='http://localhost:8000'):
        self.post = PostHandler(self)
        self.server_url = server_url
        self.connect_to_server()
        self.joint_names = ['HeadYaw',
        'HeadPitch'
        'LShoulderPitch',
        'LShoulderRoll',
        'LElbowYaw',
        'LElbowRoll',
        'LHipYawPitch',
        'LHipRoll',
        'LHipPitch',
        'LKneePitch',
        'LAnklePitch',
        'LAnkleRoll',
        'RHipYawPitch',
        'RHipRoll',
        'RHipPitch',
        'RKneePitch',
        'RAnklePitch',
        'RAnkleRoll',
        'RShoulderPitch',
        'RShoulderRoll',
        'RElbowYaw',
        'RElbowRoll']
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
        # YOUR CODE HERE
        'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],# 'LWristYaw'],
        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
        'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],#'RWristYaw']
        }

    def connect_to_server(self):
        self.server = xmlrpclib.ServerProxy(self.server_url)
        print "Client conneted to server %s\n"%str(self.server)
        return

    def list_server_methods(self):
        print "Server accepts the following methods:"
        for i in self.server.system.listMethods():
            print "     " + i
        return

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.server.get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print "sending command 'execute_keyframes'"
        self.server.execute_keyframes(keyframes)
        #self.post.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.server.set_transform(effector_name, transform)
        #self.post.set_transform(effector_name, transform)

    def relaxe(self, stiffness):
        self.server.relaxe(stiffness)

    def get_chains(self):
        print self.server.get_chains()

    def set_verbosity_level(self, level):
        self.server.set_verbosity(level)

if __name__ == '__main__':
    #agent = ClientAgent()
    print "use 'python console.py' to run prompt"
