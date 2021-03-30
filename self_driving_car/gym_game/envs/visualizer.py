from gym_game.envs.graphics import *
from gym_game.envs.entities import RectangleEntity, CircleEntity, RingEntity, SensorEntity
from gym_game.envs.agents import TextAgent  
import numpy as np

class Visualizer:
    def __init__(self, width: float, height: float, ppm: int, win: GraphWin):
        # width (meters)
        # height (meters)
        # ppm is the number of pixels per meters
        
        self.ppm = ppm
        self.display_width, self.display_height = int(width*ppm), int(height*ppm)
        self.window_created = False
        self.visualized_imgs = []
        self.car_length = 4
        self.car_width = 2
        self.win = win
        
        
        
    def create_window(self, bg_color: str = 'gray80'):
        if not self.window_created or self.win.isClosed():
            self.win = GraphWin('CARLO', self.display_width, self.display_height)
            self.win.setBackground(bg_color)
            self.window_created = True
            self.visualized_imgs = []
            return self.win
            
    def erase_car(self, agents: list):
        new_visualized_imgs = []
        # Remove the movable agents from the window
        for imgItem in self.visualized_imgs:

            if isinstance(imgItem['graphics'], list):
                for l in range(len(imgItem['graphics'])):
                    imgItem['graphics'][l].undraw()
            else:
                imgItem['graphics'].undraw()
            # else:
            #     new_visualized_imgs.append({'movable': False, 'graphics': imgItem['graphics']})

        self.visualized_imgs = new_visualized_imgs


    def update_agents(self, agents: list):
        new_visualized_imgs = []
        
        # Remove the movable agents from the window
        for imgItem in self.visualized_imgs:
            if imgItem['movable']:
                if isinstance(imgItem['graphics'], list):
                    for l in range(len(imgItem['graphics'])):
                        imgItem['graphics'][l].undraw()
                else:
                    imgItem['graphics'].undraw()
            else:
                new_visualized_imgs.append({'movable': False, 'graphics': imgItem['graphics']})
                
        # Add the updated movable agents (and the unmovable ones if they were not rendered before)
        for agent in agents:
            if agent.movable or not self.visualized_imgs:

                if isinstance(agent, RectangleEntity):
                    C = [c for c in agent.corners]
                    img = Polygon([Point(self.ppm*c.x, self.display_height-self.ppm*c.y) for c in C])
                elif isinstance(agent, TextAgent):
                    img = Text(Point(agent.anchor.x, agent.anchor.y), agent.text)
                elif isinstance(agent, CircleEntity):
                    img = Circle(Point(self.ppm*agent.center.x, self.display_height - self.ppm*agent.center.y), self.ppm*agent.radius)
                elif isinstance(agent, RingEntity):
                    img = CircleRing(Point(self.ppm*agent.center.x, self.display_height - self.ppm*agent.center.y), self.ppm*agent.inner_radius, self.ppm*agent.outer_radius)
                elif isinstance(agent, SensorEntity):
                    # Draw sensors lines
                    if(agent.name == "Front_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                                Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading))),
                                Circle(Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                            Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading), 
                                (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading)))                    

                    elif(agent.name == "Diag_Left_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [ Line(Point(agent.car.corners[0].x*self.ppm, self.display_height - agent.car.corners[0].y*self.ppm), 
                                Point(agent.car.corners[0].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/4), 
                                    (self.display_height - self.ppm*agent.car.corners[0].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/4))),
                                Circle(Point( agent.car.corners[0].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/4), 
                                    (self.display_height - self.ppm*agent.car.corners[0].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/4)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.corners[0].x*self.ppm, self.display_height - agent.car.corners[0].y*self.ppm), 
                            Point(agent.car.corners[0].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/4), 
                                (self.display_height - self.ppm*agent.car.corners[0].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/4)))
                    
                    elif(agent.name == "Diag_Right_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.corners[3].x*self.ppm, self.display_height - agent.car.corners[3].y*self.ppm), 
                            Point(agent.car.corners[3].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/4), 
                                (self.display_height - self.ppm*agent.car.corners[3].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/4))),
                            Circle(Point(agent.car.corners[3].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/4), 
                                (self.display_height - self.ppm*agent.car.corners[3].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/4)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.corners[3].x*self.ppm, self.display_height - agent.car.corners[3].y*self.ppm), 
                            Point(agent.car.corners[3].x*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/4), 
                                (self.display_height - self.ppm*agent.car.corners[3].y) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/4)))

                    elif(agent.name == "Mid_Left_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[1][0]*self.ppm, self.display_height - agent.car.edge_centers[1][1]*self.ppm), 
                            Point(agent.car.edge_centers[1][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[1][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/2))),
                            Circle(Point(agent.car.edge_centers[1][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[1][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/2)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[1][0]*self.ppm, self.display_height - agent.car.edge_centers[1][1]*self.ppm), 
                            Point(agent.car.edge_centers[1][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[1][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/2)))

                    elif(agent.name == "Mid_Right_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[3][0]*self.ppm, self.display_height - agent.car.edge_centers[3][1]*self.ppm), 
                            Point(agent.car.edge_centers[3][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[3][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/2))),
                            Circle(Point(agent.car.edge_centers[3][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[3][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/2)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[3][0]*self.ppm, self.display_height - agent.car.edge_centers[3][1]*self.ppm), 
                            Point(agent.car.edge_centers[3][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/2), 
                                (self.display_height - self.ppm*agent.car.edge_centers[3][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/2)))

                    if(agent.name == "Front_Left_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                                Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/12), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/12))),
                                Circle(Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/12), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/12)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                            Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/12), 
                                (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/12)))     

                    if(agent.name == "Front_Left_Sensor_2"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                                Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/6), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/6))),
                                Circle(Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/6), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/6)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                            Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading + np.pi/6), 
                                (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading + np.pi/6)))             
                    
                    if(agent.name == "Front_Right_Sensor"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                                Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/12), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/12))),
                                Circle(Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/12), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/12)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                            Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/12), 
                                (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/12))) 

                    if(agent.name == "Front_Right_Sensor_2"):
                        if(agent.dist_obstacle<agent.range_sensors):
                            img = [Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                                Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/6), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/6))),
                                Circle(Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/6), 
                                    (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/6)), self.ppm*0.5)]
                        else:
                            img = Line(Point(agent.car.edge_centers[0][0]*self.ppm, self.display_height - agent.car.edge_centers[0][1]*self.ppm), 
                            Point(agent.car.edge_centers[0][0]*self.ppm + self.ppm * agent.dist_obstacle * np.cos(agent.car.heading - np.pi/6), 
                                (self.display_height - self.ppm*agent.car.edge_centers[0][1]) - self.ppm * agent.dist_obstacle * np.sin(agent.car.heading - np.pi/6)))                                  



                else:
                    raise NotImplementedError
                
                if isinstance(img, list):
                    for l in range(len(img)):
                        img[l].setFill(agent.color)
                        img[l].draw(self.win)
                else:
                    img.setFill(agent.color)
                    img.draw(self.win)
                
                new_visualized_imgs.append({'movable': agent.movable, 'graphics': img})
                
        self.visualized_imgs = new_visualized_imgs

    def close(self):
        self.window_created = False
        self.win.close()
        self.visualized_imgs = []