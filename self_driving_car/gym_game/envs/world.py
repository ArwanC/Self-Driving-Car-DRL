from gym_game.envs.agents import Car, Pedestrian, RectangleBuilding, Painting
from gym_game.envs.entities import Entity
from typing import Union
from gym_game.envs.visualizer import Visualizer
from gym_game.envs.graphics import GraphWin

class World:
    def __init__(self, dt: float, width: float, height: float, win: GraphWin, ppm: float = 6):
        self.dynamic_agents = []
        self.static_agents = []
        self.t = 0 # simulation time
        self.dt = dt # simulation time step
        self.win = win
        self.visualizer = Visualizer(width, height, ppm=ppm, win=self.win)
        

        
    def add(self, entity: Entity):
        if entity.movable:
            self.dynamic_agents.append(entity)
        else:
            self.static_agents.append(entity)
        
    def tick(self):
        for agent in self.dynamic_agents:
            agent.tick(self.dt)
        self.t += self.dt
    
    def render(self, window_created: bool):
        self.visualizer.update_agents(self.agents)

    def erase_car(self):
        self.visualizer.erase_car(self.agents)
        
    @property
    def agents(self):
        return self.static_agents + self.dynamic_agents
        
    def collision_exists(self, agent = None):
        if agent is None:
            for i in range(len(self.dynamic_agents)):
                for j in range(i+1, len(self.dynamic_agents)):
                    if self.dynamic_agents[i].collidable and self.dynamic_agents[j].collidable:
                        # print(self.dynamic_agents[i], self.dynamic_agents[j].name, self.dynamic_agents[j].collidable)
                        if self.dynamic_agents[i].collidesWith(self.dynamic_agents[j]):
                            # print(self.dynamic_agents[i], self.dynamic_agents[j])
                            return True
                for j in range(len(self.static_agents)):
                    if self.dynamic_agents[i].collidable and self.static_agents[j].collidable:
                        if self.dynamic_agents[i].collidesWith(self.static_agents[j]):
                            return True
            return False
            
        if not agent.collidable: return False
        
        for i in range(len(self.agents)):
            if self.agents[i] is not agent and self.agents[i].collidable and agent.collidesWith(self.agents[i]):
                return True
        return False
    
    def car_cross_line(self):
        for car in self.dynamic_agents:
            if isinstance(car, Car):
                for painting in self.static_agents:
                    if isinstance(painting, Painting):
                        if car.obj.intersectsWith(painting.obj):
                            return True

    def close(self):
        self.reset()
        self.static_agents = []
        self.visualizer.close()
        
    def reset(self):
        self.dynamic_agents = []
        self.t = 0