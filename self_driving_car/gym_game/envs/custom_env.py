import gym
from gym import spaces
import numpy as np
from gym_game.envs.Carlo import Carlo
from gym_game.envs.graphics import GraphWin, color_rgb

class CustomEnv(gym.Env):
    #metadata = {'render.modes' : ['human']}
    def __init__(self):
        # self.create_window_once = False
        self.window_created = False
        self.ppm = 6
        self.world_width = 120 # in meters
        self.world_height = 120

        self.win = GraphWin('CARLO', self.ppm*self.world_height, self.ppm*self.world_width)
        self.win.setBackground(color_rgb(10,50,100))
        self.carlo = Carlo(self.window_created, self.win)
        self.action_space = spaces.Discrete(4) #three actions: turning left or turning right
        # Observation Space
        # Speed: 3 states : too slow (0), good speed (1) or too fast (2)
        # Sensor: 3 states : too close (0), mid distance (1) or far (2)

        self.observation_space = spaces.Box(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]), 
            np.array([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5]), dtype=np.int)

    def reset(self):
        # self.carlo.w.visualizer.close()
        del self.carlo
        self.carlo = Carlo(self.window_created, self.win)
        self.window_created = True
        obs = self.carlo.observe()
        return obs

    def step(self, action):
        self.carlo.action(action)
        obs = self.carlo.observe()
        reward = self.carlo.evaluate()
        done = self.carlo.is_done()
        return obs, reward, done, {}

    def render(self, mode="human", close=False):
        self.carlo.view(self.window_created)