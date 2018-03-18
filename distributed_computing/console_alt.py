import sys
import os
from agent_client import ClientAgent
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from numpy.matlib import identity
from keyframes import *
from time import sleep

class Console:
     def __init__(self, client_agent=ClientAgent()):
         self.agent=client_agent

     def comunicate(self):
         agent = self.agent
         while(True):
             command = raw_input("\nEnter a command or 'help': ")
             cmds = command.split(" ")
             print str(cmds)
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
                 print "\n help, exit, break"
             elif cmds[0] == 'quit' or cmds[0] == 'exit':
                break
             #elif cmds[0] == '\x1b':
            #    print "history"
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
                     print " Usage: ./agent_client execute_keyframes keyframe_filename"
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
                             agent.relaxe(cmds[2])
                         else:
                             agent.relaxe(0.0)  # turn off joints
                         sleep(5)               #wait nao to fall
                         agent.relaxe(1)
                         pass
                     else:
                         print " Keyframes not found"
                         print " Available keyfrmames are: relaxe [stifness], hello, leftBackToStand, leftBellyToStand, righBackToStand, rightBellyToStand, wipe_forehead"

                 if len(cmds)>2:
                     print " executing keyframe %s .."%cmds[1]
                     agent.execute_keyframes(cmds[2])
             else:
                 print " Usage: ./agent_client [action] [parameter1] [parametern..]"
                 print " Try './agent_client help' for more options"

if __name__ == '__main__':
    console = Console()
    console.comunicate()
