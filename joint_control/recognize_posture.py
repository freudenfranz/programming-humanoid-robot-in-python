'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBellyToStand
import pickle
import numpy as np
from os import listdir, path


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        ROBOT_POSE_CLF = 'robot_pose.pkl'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF))  # LOAD YOUR CLASSIFIER
        #TODO: close file??

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        ROBOT_POSE_DATA_DIR = 'robot_pose_data'
        possible_postures = listdir(ROBOT_POSE_DATA_DIR)
        posture = 'unknown'
        ''' YOUR CODE HERE
        the features (e.g. each row of the data) are ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'AngleX', 'AngleY'], where 'AngleX' and 'AngleY' are body angle (e.g. Perception.imu) and others are joint angles.'''
        features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        unknown_posture = []
        for feature in features:
            unknown_posture.append(perception.joint.get(feature))
        unknown_posture += perception.imu
        unknown_posture = np.array(unknown_posture)
        #print unknown_posture
        posture_index = self.posture_classifier.predict(unknown_posture.reshape(1,-1))
        recognized_posture = possible_postures[posture_index[-1]]
        print recognized_posture
        return recognized_posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.set_keyframes(leftBellyToStand())  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
