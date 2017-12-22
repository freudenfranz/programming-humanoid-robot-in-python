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
from keyframes import *
from numpy.matlib import identity
class PostHandler(object):
    '''the post handler wraps functions to be executed in parallel
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    #print "Connected to Server:"
    #print server.__doc__
    #print "Available methods ared:"
    #print self.server.system.listMethods()
    #print "---------------------------"

    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpclib.ServerProxy('http://localhost:8000')

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
        self.server.execute_keyframes(keyframes)
        self.server.run()

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.server.get_transform()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.server.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    while(True):
        command = raw_input("\nEnter a command or 'help': ")
        cmds = command.split(" ")
        #print str(cmds)
        if not cmds:
            cmds = 'emtpy'
        if cmds[0] == 'help':
            print "Available commands are:"
            print " set_transform effector_name transform"
            print " set_angle joint_name angle"
            print " get_angle joint_name\n"
            print " get_posture"
            print " get_transform name\n"
            print " execute_keyframes keyframe-name"
        elif cmds[0] == 'set_transform':
            T = identity(4)
            T[-1, 1] = 0.05
            T[-1, 2] = 0.26
            if len(cmds)<3:
                print " Usage: ./agent_client set_tranform effector_name transformation-matrix"
                print " No parametres given. Using default: 'LLeg', and matrix\n %s"%str(T)
                cmds.append("LLeg")
                cmds.append(T)
            agent.set_transform(cmds[1], cmds[2])
        elif cmds[0] == 'set_angle':
            if len(cmds)<3:
                print " Usage: ./agent_client set_angle joint_name angle"
                print " Not enought parametres given. Using default: 'HeadYaw', '0.707'"
                cmds.append("HeadYaw")
                cmds.append(0.707)
            agent.set_angle(cmds[1], cmds[2])
            print " Angle %s set to %s"%(cmds[1], agent.get_angle(cmds[1]))
        elif cmds[0] == 'get_angle':
            if len(cmds)<2:
                print " Usage: ./agent_client get_angle joint_name"
                print " No joint_name given. Using default: 'HeadYaw'"
                cmds.append("HeadYaw")
            print " Angle of %s is: %s"%(cmds[1], agent.get_angle(cmds[1]))
        elif cmds[0] == 'get_posture':
            print " Actual posture is: %s"%agent.get_posture()
        elif cmds[0] == 'get_transform':
            print " Transform of %s is:\n%s"%(command[1], agent.get_transform(command[1]))
        elif cmds[0] == 'execute_keyframes':
            if len(cmds)<2:
                print " Usage: ./agent_client execute_keyframes keyframe_filename)"
                print " No keyframe given. Using default: 'hello'"
                cmds.append(agent.execute_keyframes(hello()))
            else:
                if cmds[1] == 'hello':
                    cmds.append(hello())
                elif cmds[1] == 'leftBackToStand':
                    cmds.append(leftBackToStand())
                elif cmds[1] == 'leftBellyToStand':
                    cmds.append(leftBellyToStand())
                elif cmds[1] == 'righBackToStand':
                    cmds.append(rightBackToStand())
                elif cmds[1] == 'rightBellyToStand':
                    cmds.append(rightBellyToStand())
                elif cmds[1] == 'wipe_forehead':
                    cmds.append(wipe_forehead())
                elif cmds[1] == 'relaxe':
                    if len(cmds)>2:
                        agent.action.stiffness = {j: cmds[2] for j in agent.joint_names}
                    else:
                        agent.action.stiffness = {j: 0.2 for j in agent.joint_names}  # turn off joints
                    time.sleep(5)                                       #wait nao to fall
                    action.stiffness = {j: 1 for j in agent.joint_names} #turn on number_of_joints
                else:
                    print " Keyframes not found"
                    print " Available keyfrmames are: relaxe [stifness], hello, leftBackToStand, leftBellyToStand, righBackToStand, rightBellyToStand, wipe_forehead"

            if len(cmds)>2:
                print " executing keyframe %s .."%cmds[1]
                agent.execute_keyframes(cmds[2])
        else:
            print " Usage: ./agent_client [action] [parameter1] [parametern..]"
            print " Try './agent_client help' for more options"
    '''
    print "set angle to 0.707"
    agent.set_angle("HeadYaw", 0.707)
    print "read previous setted angle"
    print "angle is: %s"%agent.get_angle("HeadYaw")
    print "read posture.."
    print "posture is: %s"%agent.get_posture()
    print "execute keyframes to 'hello'"

    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    print "set effector 'LLeg' to y=0.05, z=0"
    agent.set_transform('LLeg', T)
    print "transform is: %s"%agent.get_transform('LLeg')'''
