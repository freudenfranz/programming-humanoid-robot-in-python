
import numpy as np
import unittest
from forward_kinematics import ForwardKinematicsAgent


class Tests(unittest.TestCase):
    def single_matrix(self, agent, joint_name, angle_in_radians, result_matrix):
        self.assertTrue(np.allclose(agent.local_trans(joint_name, angle_in_radians), result_matrix))

    def test_local_trans(self):
        agent = ForwardKinematicsAgent()

        A = np.identity(4)
        self.single_matrix(agent, 'HeadJaw', 1, np.identity(4))

        A = [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        self.single_matrix(agent, 'HeadJaw', 1, A)

        self.single_matrix(agent, 'HeadJaw', 0, [[0, 0, 0, 0],
                                                 [0, 0, 0, 0],
                                                 [0, 0, 0, 0],
                                                 [0, 0, 126.5, 1]])

    #def test_whole_forward_kinematics(self):
    #    pass


if __name__ == '__main__':
    unittest.main()

