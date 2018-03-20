'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
from SimpleXMLRPCServer import SimpleXMLRPCServer
from inverse_kinematics import InverseKinematicsAgent
from math import pi
from threading import Thread
from keyframes import hello

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, addr='localhost', port=8000):
        super(InverseKinematicsAgent, self).__init__()
        print "start Server.."
        self.server = SimpleXMLRPCServer((addr, port))
        print "server startet: %s"%repr(self.server)
        self.verbosity_level = 0
        self.server.register_function(self.get_angle)
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)
        self.server.register_function(self.relaxe)
        self.server.register_function(self.start_server)
        self.server.register_function(self.run)
        self.server.register_function(self.get_chains)
        self.server.register_function(self.set_verbosity)
        self.server.register_introspection_functions()


    def start_server(self):
        print "start serving!"
        self.server_thread = Thread(target=self.server.serve_forever)
        self.server_thread.start()
        print "server is serving in thread %s"%str(self.server_thread)

    def stop_server(self):
        print "stopping agent.."
        self.agent_should_run = False
        print "shutting server down.."
        self.server.server_close()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        if self.verbosity_level > 5:
            print "getting angle of %s.."%joint_name
        return self.perception.joint.get(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
            @angle in Grad
        '''
        # YOUR CODE HERE
        if self.verbosity_level > 3:
            print "trying to set %s to %s degrees"%(joint_name, angle)
        if joint_name in self.perception.joint:
            self.target_joints[joint_name] = float(angle) * pi / 180.0;
            if self.verbosity_level > 4:
                print "%s is now at %s degrees"%(joint_name, self.perception.joint[joint_name])
        else:
            raise NameError ("joint_name %s not found in joints")%str(joint_name)

        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)



    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print "executing keyframes.."
        self.set_keyframes(keyframes)
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print str(self.transforms[name])
        return True

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the'''
        # YOUR CODE HERE
        if self.verbosity_level > 3:
            print "trying to set %s to %s"%(effector_name,transform)
        self.set_transforms(effector_name, transform)
        if self.verbosity_level > 4:
            print "transformation matrix of %s is now %s"%(effector_name, self.get_transform(effector_name,transform))
        return True

    def get_chains(self):
        return self.chains

    def relaxe(self, stiffness):
        self.stiffness = {j: stiffness for j in self.joint_names}
        return True

    def set_verbosity(self, level):
        self.verbosity_level = level
        if self.verbosity_level > 1:
            print "verbosity level set to %s"%self.verbosity_level
        return True

if __name__ == '__main__':
    print "agent_server seems to be __main__"
    agent = ServerAgent()
    agent.start_server()
    agent.run()
    agent.stop_server()
