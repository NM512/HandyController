from robogym.envs.dactyl.locked import make_env
from gym import ActionWrapper
import gym
from HandyController.hand_pose_detector import HandPoseDetector
from HandyController.key_detector import KeyDetector
from pynput.keyboard import Key
import time

""" Action space in lunar lander environment
0:jet injection for underside, 1:jet injection for side
"""

class LunarLanderWrapper(ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.hand_det = HandPoseDetector(render_img=True, render_3d=False)

    def angles_to_action(self, action, angles):
        action[0] = min(angles['index']['second'] * 2.0, 2.0) - 1.0
        action[1] = min(angles['middle']['second'] * 2.0, 2.0) - 1.0
        return action

    def action(self, action):
        positions, angles = self.hand_det.detect()
        action = self.angles_to_action(action, angles)
        return action


if __name__ == '__main__':
    env = gym.make("LunarLanderContinuous-v2")
    env = LunarLanderWrapper(env)
    obs = env.reset()
    print('Action shape', env.action_space)
    while True:
        action = [0.0] * env.action_space.shape[0]
        obs, reward, done, info = env.step(action)
        env.render('human')
        time.sleep(0.1)
        if done:
            observation = env.reset()