'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import *
from os import listdir

class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        # YOUR CODE
        names = list()
        times = list()
        keys = list()
        do_nothing = (names, times, keys)

        if posture == 'Back':
            self.set_keyframes(leftBackToStand())
        elif posture == 'Belly':
            self.set_keyframes(leftBellyToStand())
        elif posture == 'Crouch':
            self.set_keyframes(do_nothing)
        elif posture == 'Frog':
            self.set_keyframes(do_nothing)
        elif posture == 'HeadBack':
            self.set_keyframes(do_nothing)
        elif posture == 'Knee':
            self.set_keyframes(do_nothing)
        elif posture == 'Left':
            self.set_keyframes(leftBackToStand())
        elif posture == 'Right':
            self.set_keyframes(rightBackToStand())
        elif posture == 'Sit':
            self.set_keyframes(do_nothing)
        elif posture == 'Stand':
            self.set_keyframes(wipe_forehead(0))
        elif posture == 'Standinit':
            self.set_keyframes(do_nothing)
        else:
            self.set_keyframes(do_nothing)


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent = StandingUpAgent()
    agent.run()
