from robogym.envs.dactyl.locked import make_env
from gym import ActionWrapper
from HandyController.hand_pose_detector import HandPoseDetector
from HandyController.key_detector import KeyDetector
from pynput.keyboard import Key

""" Action space in robotgym dactyl environment
0:WRZ, 1:WRY,
2:I1Z, 3:I1Y, 4:I2Y,
5:M1Z, 6:M1Y, 7:M2Y,
8:R1Z, 9:R1Y, 10:R2Y,
11:P0Y, 12:P1Z, 13:P1Y, 14:P2Y,
15:T1R, 16:T1Y, 17:T2Z, 18:T2Y, 19:T3Y
"""

class RobogymDactylWrapper(ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.key_det = KeyDetector((Key.f1, Key.f2, Key.f3, Key.f4))
        self.hand_det = HandPoseDetector(render_img=True, render_3d=False)
        self.min_max = {'thumb':{'first':[0.0, 1.0], 'second':[0.0, 1.0], 'third':[0.0, 1.0]},
                        'index':{'first':[0.0, 1.0], 'second':[0.0, 1.0], 'third':[0.0, 1.0]},
                        'middle':{'first':[0.0, 1.0], 'second':[0.0, 1.0], 'third':[0.0, 1.0]},
                        'ring':{'first':[0.0, 1.0], 'second':[0.0, 1.0], 'third':[0.0, 1.0]},
                        'pinky':{'first':[0.0, 1.0], 'second':[0.0, 1.0], 'third':[0.0, 1.0]}}

        self.indexes = {'thumb':{'first':0, 'second':1, 'third':0},
                        'index':{'first':0, 'second':0, 'third':0},
                        'middle':{'first':0, 'second':0, 'third':0},
                        'ring':{'first':0, 'second':0, 'third':0},
                        'pinky':{'first':0, 'second':0, 'third':0}}

        self.wrist_pose = [5.0, 5.0]

    def angles_to_action(self, action, angles):

        action[2] = 10 - min(int(angles['index']['z'] * 100.0), 10)
        action[3] = min(int(angles['index']['third'] * 10.0), 10)
        action[4] = min(int(angles['index']['second'] * 10.0), 10)
        action[5] = 10 - min(int(angles['middle']['z'] * 100.0), 10)
        action[6] = min(int(angles['middle']['third'] * 10.0), 10)
        action[7] = min(int(angles['middle']['second'] * 10.0), 10)
        action[8] = 10 - min(int(angles['ring']['z'] * 100.0), 10)
        action[9] = min(int(angles['ring']['third'] * 10.0), 10)
        action[10] = min(int(angles['ring']['second'] * 10.0), 10)
        action[11] = 0
        action[12] = 10 - min(int(angles['pinky']['z'] * 100.0), 10)
        action[13] = min(int(angles['pinky']['third'] * 10.0), 10)
        action[14] = min(int(angles['pinky']['second'] * 10.0), 10)
        action[16] = min(int(angles['thumb']['second'] * 10.0), 10)
        action[17] = min(int(angles['thumb']['z'] * 10.0), 10)
        # direction is reversed?
        action[19] = 10 - min(int(angles['thumb']['first'] * 10.0), 10)
        return action

    def key_to_action(self, action, key_flags):
        if key_flags[0]:
            self.wrist_pose[0] = min(self.wrist_pose[0] + 0.1, 10.0)
        if key_flags[1]:
            self.wrist_pose[0] = max(self.wrist_pose[0] - 0.1, 0.0)
        if key_flags[2]:
            self.wrist_pose[1] = min(self.wrist_pose[1] + 0.1, 10.0)
        if key_flags[3]:
            self.wrist_pose[1] = max(self.wrist_pose[1] - 0.1, 0.0)

        action[0] = int(self.wrist_pose[0])
        action[1] = int(self.wrist_pose[1])
        return action

    def action(self, action):
        key_flags = self.key_det.read_keys()
        positions, angles = self.hand_det.detect()
        action = self.angles_to_action(action, angles)
        action = self.key_to_action(action, key_flags)
        return action


if __name__ == '__main__':
    env = make_env()
    env = RobogymDactylWrapper(env)
    obs = env.reset()
    print('Action shape', env.action_space.shape)
    while True:
        # action is defined as discrete using gym.spaces.MultiDiscrete(value has to be 0~10)
        action = [5] * env.action_space.shape[0]
        obs, reward, done, info = env.step(action)
        env.render('human')
