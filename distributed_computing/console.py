import sys
import os
from cmd import Cmd
from agent_client import ClientAgent
from agent_server import ServerAgent
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import *
from numpy.matlib import matrix, identity
from math import pi
from time import sleep
from threading import Thread
from subprocess import Popen, PIPE
from pydoc import help
import traceback

class Console(Cmd):
    def __init__(self, start_simspark=True, server_agent=True, client_agent=True):
        #''',server_agent=ServerAgent()'''):
        #self.simspark = Thread(target=self.start_simspark)
        #self.simspark.start()
        if start_simspark:
            print"starting simspark.."
            self.simspark=Popen(['simspark'], stdout=PIPE, stderr=PIPE)
            sleep(2)
        if server_agent:
            self.server_agent=ServerAgent()
            self.server_agent.start_server
            self.server_agent.start()
            sleep(2)
        if client_agent:
            self.agent=ClientAgent()

        self.prompt = 'nao> '
        self.traceback = None
        Cmd.__init__(self)

    def default(self, line):
        '''defailt error messege if a comande is not known'''
        print('What do you mean by "{0}"? If you need \'help\' then tell me so!'.format(line))
    '''
    def preloop(self):
        pass
    def postloop(self):
        pass
    def precmd(self, line):
        pass
    def postcmd(self, stop, line):
        pass
    '''
    def do_q(self, arg):
        '''Leaves this comand line interpreter'''
        self.do_quit(arg)
        return True

    def do_quit(self, arg):
        '''Leaves this comand line interpreter'''
        self.do_exit(arg)
        return True

    def do_exit(self,arg):
        '''Leaves this comand line interpreter'''
        print   '''\n Oo_oO o   You are leaving my interpreter now!\n'''\
                '''   |--/  \n'''\
                ''' _| |_   \n'''
        try:
            self.server_agent.stop_server()
            print "Server stopped.."
        except AttributeError:
            print "Server, if there is one, runs in an extern process. Don't forget to shut it down!"

        try:
            self.simspark.kill()
            print"Simspark stopped.."
        except AttributeError:
            print "Simspark, if on, doesn't run in this interpreter. Don't forget to shut it down!"

        return True

    def do_start_serveragent(self, args):
        '''Starts a server agent thread and connects this console over the client agent to it'''
        try:

            self.server_agent=ServerAgent()
            self.server_agent.start_server()
            self.server_agent.start()
            self.agent.get_verbosity_level()
            sleep(5)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print message
        return False

    def do_simspark(self, args):
        '''Starts a simspark-simulator in a new window'''
        try:
            self.simspark=Popen(['simspark'], stdout=PIPE, stderr=PIPE)
            sleep(2)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print message
        return False

    def do_list_server_abilities(self,args):
        '''Lists all methods the server is providing.'''
        self.agent.list_server_methods()
        return False

    def do_print_traceback(self, args):
        '''Prints the traceback of the last Exception'''
        print self.traceback
        return False

    def do_set_verbosity(self, args):
        '''Sets verbosity level, wich effects the amount of information outputted by the server'''
        self.agent.set_verbosity_level(args[0])
        return False

    def do_get_verbosity(self, args):
        '''Prints the actual verbosity_level'''
        self.agent.get_verbosity_level()
        return False

    def do_clear(self, args):
        clear = "\n" * 100
        jmp_back = "\r" * 100

        print clear

        return False

    def do_reload_agent(self, args):
        ''''to speed up the developement-loop with this
        you can reload the agent while the simulation is still
        running if you changed the code live'''
        self.agent.reload_agent()
        return False


    #==========Real Nao commands start here===========#
    def do_set_transform(self, args):
        '''\n Sets the transformation matrix of a effector, which will lead to its moovement.\n\n'''\
        '''  Usage: set_transform <effector_name> <x-position> <y-position> <z-position> <roll> <pitch> <yaw>\n'''\
        '''  Where: (x,y,z) is the point seen from the Torso of the effector \n'''\
        '''         (roll,pitch, yaw) are the ortientation-angles of the effector in degrees \n'''\
        '''  If you don't any parameters, the stanandig Leg will be used:\n'''\
        '''         LLeg, x=0mm, y=50mm, z=-333.09mm, roll=0deg, pitch=0deg, yaw=0deg\n'''\
        '''  Possible effectors are:\n'''\
        '''   'Head' 'LArm' 'LLeg' 'RLeg' 'RArm' \n'''
        args = args.split()
        if len(args)==0:
            print "No arguments specified. Using standard values:"
            print "LLeg, x=0mm, y=50mm, z=-333.09mm, roll=0deg, pitch=0deg, yaw=0deg"
            try:
                self.agent.set_transform('LLeg', 0, 50, -333.09, 0, 0, 0)
            except:
                traceback.print_exc()
        #TODO Fuer richtige Eingaben implementieren
            '''elif len(args)==3:
                dx=float(args[0])
                dy=float(args[1])
                dz=float(args[2])
                print "\tValues of translation-verctor are x=%.2f, y=%.2f, z=%.2f"%(dx, dy,dz)
                try:
                    self.agent.set_transform('LLeg',dx,dy,dz)
                except:
                    print "<Exeption>"
                    print("Nao thinks that 'set_tranform' function does not work properly")
                    traceback.print_exc()
            '''
        else:
            print self.do_set_transform.__doc__

        return False

    def do_get_transform(self, args):
        '''\n Returns the actual transformation matrix of a joint or of all joints of an effector.\n\n'''\
        '''  Usage: get_transform <[joint_name, effector_name]>\n'''\
        '''  Possible effectors are:\n'''\
        '''   'Head' 'LArm' 'LLeg' 'RLeg' 'RArm' \n'''\
        '''  Possible joints are: \n'''\
        '''   'HeadYaw', 'HeadPitch' \n'''\
        '''   'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'\n'''\
        '''   'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'\n'''\
        '''   'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'\n'''\
        '''   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'\n'''

        args = args.split()
        if len(args) == 1:
            #if an effector is given, print matrixes of all joints'''
            chains = self.agent.chains
            if args[0] in chains:
                for joint in chains[args[0]]:
                    self.do_get_transform(joint)
            #if a joint is given, print transformation matrix of it'''
            else:
                try:
                    print("\n%s: "%args[0])
                    self.agent.get_transform(args[0])
                except KeyError:
                    print("Joint name not known!")
                    print self.do_get_transform.__doc__
                except:
                    print("It seems, that the function 'execute_keyframes' doesn't work properly")
                    print self.do_get_transform.__doc__
                    print traceback.format_exc()
                    #traceback.print_exc()
        else:
            print "Incomplete input!"
            print self.do_get_transform.__doc__

        return False

    def do_set_angle(self, args):
        '''\n Sets the angle of a joint given in degrees, which will lead to its moovement.\n\n'''\
        '''  Usage: set_angle <joint_name> <angle>\n'''\
        '''   If you don't use any parameters it will be uses 'HeadYaw' and '0.707'\n'''\
        '''  Possible joints are:\n'''\
        '''   "HeadYaw" "HeadPitch" \n'''\
        '''   "LShoulderPitch" "LShoulderRoll" "RShoulderPitch" "RShoulderRoll"\n'''\
        '''   "LElbowYaw" "LElbowRoll" "RElbowYaw" "RElbowRoll" \n'''\
        '''   "LWristYaw" "RWristYaw" \n'''\
        '''   "LHipYawPitch" "LHipPitch" "LHipRoll" "RHipRoll" "RHipPitch" "RHipYawPitch"\n'''\
        '''   "LKneePitch" "RKneePitch"\n'''\
        '''   "LAnklePitch" "LAnkleRoll" "RAnklePitch" "RAnkleRoll" \n'''

        args = args.split()
        if len(args)<2:
            print "No joint nor anle specified!"
            print self.do_set_angle.__doc__

        elif len(args)==2:
            try:
                self.agent.set_angle(args[0], args[1])
            except AttributeError:
                print "joint_name or angle is not right"
            except:
                print("set_angle function seems not to work properly!")
                traceback.print_exc()
        else:
            print self.do_set_angle.__doc__

        return False

    def do_get_angle(self, args):
        '''\n Returns the actual angle of a joint or a joint-chain\n\n'''\
        '''  Usage: get_angle <[joint_name, chain-name, all]> \n\n'''\
        '''  Possible chain are:\n'''\
        '''   "Head" "LArm" "LLeg" "RLeg" "RArm" \n'''\
        '''  Possible joints are:\n'''\
        '''   "HeadYaw" "HeadPitch" \n'''\
        '''   "LShoulderPitch" "LShoulderRoll" "RShoulderPitch" "RShoulderRoll"\n'''\
        '''   "LElbowYaw" "LElbowRoll" "RElbowYaw" "RElbowRoll" \n'''\
        '''   "LWristYaw" "RWristYaw" \n'''\
        '''   "LHipYawPitch" "LHipPitch" "LHipRoll" "RHipRoll" "RHipPitch" "RHipYawPitch"\n'''\
        '''   "LKneePitch" "RKneePitch"\n'''\
        '''   "LAnklePitch" "LAnkleRoll" "RAnklePitch" "RAnkleRoll" \n'''
        '''   "all for all joints" '''
        args = args.split()
        if len(args) == 1:
            try:
                if args[0] == 'all':
                    for i in self.agent.joint_names:
                        self.do_get_angle(i)
                elif args[0] == 'Head':
                    self.do_get_angle("HeadYaw")
                    self.do_get_angle("HeadPitch")
                elif args[0] == 'LArm':
                    self.do_get_angle("LShoulderPitch")
                    self.do_get_angle("LShoulderRoll")
                    self.do_get_angle("LElbowYaw")
                    self.do_get_angle("LElbowRoll")
                    #self.do_get_angle("LWristYaw")
                elif args[0] == 'LLeg':
                    self.do_get_angle("LHipYawPitch")
                    self.do_get_angle("LHipPitch")
                    self.do_get_angle("LHipRoll")
                    self.do_get_angle("LKneePitch")
                    self.do_get_angle("LAnklePitch")
                    self.do_get_angle("LAnkleRoll")
                elif args[0] == 'RArm':
                    self.do_get_angle("RShoulderPitch")
                    self.do_get_angle("RShoulderRoll")
                    self.do_get_angle("RElbowYaw")
                    self.do_get_angle("RElbowRoll")
                    #self.do_get_angle("RWristYaw")
                elif args[0] == 'RLeg':
                    self.do_get_angle("LHipYawPitch")
                    self.do_get_angle("LHipPitch")
                    self.do_get_angle("LHipRoll")
                    self.do_get_angle("LKneePitch")
                    self.do_get_angle("LAnklePitch")
                    self.do_get_angle("LAnkleRoll")
                else:
                    angle =self.agent.get_angle(args[0])
                    print "%s:\t\t %.3fdeg =%.2frad"%(args[0], angle / pi * 180, angle)

            except:
                print("get_angle function seems not to work properly")
        else:
            print self.do_get_angle.__doc__

        return False

    def do_get_posture(self, args):
        '''\n Returns the actual recognized posture of nao.\n\n'''\
        '''  Usage: get_posture \n'''
        if len(args)==0:
            try:
                print self.agent.get_posture()
            except Exception, err:
                print "<<Exeption>>"
                print "Function 'get_posture' doesn't seem to work properly"
                traceback.print_exc()

        else:
            print self.do_get_posture.__doc__

        return False

    def do_execute_keyframes(self, args):
        '''\n Initiates a moovement represented by keyframes.\n\n'''\
        '''  Usage: execute_keyframes [keyframes_name] \n\n'''\
        '''  If no keyframes are specified, the precalculated and \n'''\
        '''      stored with 'set_keyframes()' ones are taken\n'''\
        '''  Known keyframes stored in keyframe folder are:\n'''\
        '''    leftBackToStand\n'''\
        '''    wipe_forehead\n'''\
        '''    rightBellyToStand\n'''\
        '''    leftBellyToStand\n'''\
        '''    rightBackToStand\n'''\
        '''    hello'''

        args = args.split()
        action = {}
        if len(args) == 1 or len(args) == 2:
            if args[0] == 'hello':
                action = hello()
            elif args[0] == 'leftBackToStand':
                action = leftBackToStand()
            elif args[0] == 'leftBellyToStand':
                action = leftBellyToStand()
            elif args[0] == 'righBackToStand':
                action = rightBackToStand()
            elif args[0] == 'rightBellyToStand':
                action = rightBellyToStand()
            elif args[0] == 'wipe_forehead':
                action = wipe_forehead('bla')
            elif args[0] == 'relaxe':
                if len(args)==2:
                    self.agent.relaxe(args[2])
                else:
                    self.agent.relaxe(0.0)  # turn off joints
                sleep(5)               #wait nao to fall
                self.agent.relaxe(1)
                pass
            else:
                if self.do_get_verbosity > 2:
                    print "Keyframes-name not known"
                    print " use 'help execute_keyframes' for more information"
                #TODO just execute precalculated keyframes??

            try:
                self.agent.execute_keyframes(action)
            except:
                print"Nao thinks, function 'execute_keyframes' doesn't work properly"
                traceback.print_exc()
        if len(args) == 0:
            try:
                print"No keyframes specified. Takeing the prestored ones!"
                self.agent.execute_keyframes(action)

            except:
                self.traceback = traceback.format_exc()
                print"Error!"
                print"Did you prestore any keyframes?"
                print " use 'help execute_keyframes' for more information"
                print"show last traceback witch 'print_traceback'"
        else:
            print self.do_execute_keyframes.__doc__

        return False

    def do_get_keyframes(self, args):
        '''prints the servers actual keyframes'''
        try:
            self.agent.get_keyframes()
        except:
            self.traceback = traceback.format_exc()
            print"Error: Do you have a serveragent running?"
            print"use print_traceback for more information"
            print self.do_get_keyframes.__doc__
        return False

    def do_list_effector_chains(self, args):
        '''Lists all joints-chains for all effectors\n'''\
        '''A joint-chain for an effector contains all joints for whome angles would have to'''\
        ''' be calculated if this effector would have to be mooved'''
        print self.agent.get_chains()
        return False

if __name__ == '__main__':
    string = '''
    ##############################################################
    ## Oo_oO   Hi I'm Nao. Welcome my interpreter!              ##
    ## --|--   If you have questions,                           ##
    ## _| |_   tell me to 'help' or just make a '?'             ##
    ##############################################################

    TODO's:
    set_transform funktioniert noch nicht:
          File "/usr/lib/python2.7/xmlrpclib.py", line 800,
          in close raise Fault(**self._stack[0])
          xmlrpclib.Fault: <Fault 1:
          "<type 'exceptions.NameError'>:
          global name 'foot2hip' is not defined">
          --> alles muss noch  fertig gemacht werden.
    get_transform ueberpruefen sobald set_transform funktioniert
    get_transform raised die falsche exeption..
    get_angle all scheint eine fehlendes gelenk zu finden
    checken ob eingaben Richtig sind (joint.. names)
    clr jmp_back funktioniert nicht
    forward_kinematics hint 3 torso machen.
    execute keyframes relaxe und right back to stand
        funktioniert noch nicht
    set_transform funkitoniert jetzt mit x,y,z sollte aber
        ueber eine Matrix laufen..
    set_verbosity die Zahl muss ueberprueft werden. bei leer stuerzt
        es ab
    -implement to start multiple server_agents
    '''

    console = Console(False, False, True)
    console.cmdloop(string)
