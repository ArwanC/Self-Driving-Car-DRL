from gym_game.envs.world import World

#from gym_game.envs.agents import Car, RingBuilding, CircleBuilding, Painting, Pedestrian, Sensors, getEquidistantPoints, TextAgent, CheckPointAgent
from gym_game.envs.agents import *
from gym_game.envs.geometry import Point, Line
from gym_game.envs.graphics import GraphWin

import time
from tkinter import *
import numpy as np
import threading
import random
from math import sqrt

human_controller = False

dt = 0.1 # time steps in terms of seconds. In other words, 1/dt is the FPS.
world_width = 120 # in meters
world_height = 120
inner_building_radius = 25
num_lanes = 3
lane_marker_width = 0.5
num_of_lane_markers = 20
lane_width = 5

car_length = 4
car_width = 2


speed_limit = 5
speed_minimum = 1

def speed_state(car):
    if car.speed < speed_minimum:
        return 0
    elif car.speed > speed_limit:
        return 2
    else:
        return 1

distance_far = 7
distance_close = 3

def sensor_state_obstacle(sensor):
    if sensor.dist_obstacle < distance_close:
        return 0
    elif sensor.dist_obstacle > distance_far:
        return 2
    else:
        return 1

def sensor_state_painting(sensor):
    if sensor.closest_painting < distance_close:
        return 0
    elif sensor.closest_painting > distance_far:
        return 2
    else:
        return 1

def distance_state(car, checkpoint, init_dist):
    dist = car.distanceTo(checkpoint)
    for i in range(1, 6):
        if dist <= (init_dist/5)*i:
            return i
    return 5

class Carlo:
    def __init__(self, window_created: bool, win: GraphWin):

        self.window_created = window_created
        self.win = win
        self.w = World(dt, width = world_width, height = world_height, win = self.win, ppm = 6) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

        # self.intersect_world()
        self.random_checkpoint_world()
        self.reward = 0

        
        # Sensors initialization
        self.range_sensors = 20 # 10 meters
        self.precision = 10 # sensors' resolution

        self.s1 = Sensors(
            Point(self.c1.edge_centers[0][0]*self.ppm, self.c1.edge_centers[0][1]*self.ppm), 
            Point(self.c1.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading), 
                (self.ppm*self.c1.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading)), 
            self.range_sensors,
            "Front_Sensor",
            self.c1,
            self.precision)
        

        self.s2 = Sensors( 
            Point(self.c1.corners[0].x*self.ppm, self.c1.corners[0].y*self.ppm), 
            Point(self.c1.corners[0].x*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading + np.pi/4), 
                (self.ppm*self.c1.corners[0].y) + self.ppm * self.range_sensors * np.sin(self.c1.heading + np.pi/4)), 
            self.range_sensors,
            "Diag_Left_Sensor",
            self.c1,
            self.precision)

        self.s3 = Sensors(
            Point(self.c1.corners[3].x*self.ppm, self.c1.corners[3].y*self.ppm), 
            Point(self.c1.corners[3].x*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading - np.pi/4), 
                (self.ppm*self.c1.corners[3].y) + self.ppm * self.range_sensors * np.sin(self.c1.heading - np.pi/4)), 
            self.range_sensors,
            "Diag_Right_Sensor",
            self.c1,
            self.precision)

        self.s4 = Sensors(
            Point(self.c1.edge_centers[1][0]*self.ppm, self.c1.edge_centers[1][1]*self.ppm), 
            Point(self.c1.edge_centers[1][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading + np.pi/2), 
                (self.ppm*self.c1.edge_centers[1][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading + np.pi/2)), 
            self.range_sensors,
            "Mid_Left_Sensor",
            self.c1,
            self.precision)

        self.s5 = Sensors(
            Point(self.c1.edge_centers[3][0]*self.ppm, self.c1.edge_centers[3][1]*self.ppm), 
            Point(self.c1.edge_centers[3][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading - np.pi/2), 
                (self.ppm*self.c1.edge_centers[3][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading - np.pi/2)), 
            self.range_sensors,
            "Mid_Right_Sensor",
            self.c1,
            self.precision)

        self.s6 = Sensors(
            Point(self.c1.edge_centers[0][0]*self.ppm, self.c1.edge_centers[0][1]*self.ppm), 
            Point(self.c1.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading + np.pi/12), 
                (self.ppm*self.c1.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading + np.pi/12)), 
            self.range_sensors,
            "Front_Left_Sensor",
            self.c1,
            self.precision)

        self.s7 = Sensors(
            Point(self.c1.edge_centers[0][0]*self.ppm, self.c1.edge_centers[0][1]*self.ppm), 
            Point(self.c1.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading + np.pi/6), 
                (self.ppm*self.c1.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading + np.pi/6)), 
            self.range_sensors,
            "Front_Left_Sensor_2",
            self.c1,
            self.precision)

        self.s8 = Sensors(
            Point(self.c1.edge_centers[0][0]*self.ppm, self.c1.edge_centers[0][1]*self.ppm), 
            Point(self.c1.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading - np.pi/12), 
                (self.ppm*self.c1.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading - np.pi/12)), 
            self.range_sensors,
            "Front_Right_Sensor",
            self.c1,
            self.precision)

        self.s9 = Sensors(
            Point(self.c1.edge_centers[0][0]*self.ppm, self.c1.edge_centers[0][1]*self.ppm), 
            Point(self.c1.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.c1.heading - np.pi/6), 
                (self.ppm*self.c1.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.c1.heading - np.pi/6)), 
            self.range_sensors,
            "Front_Right_Sensor_2",
            self.c1,
            self.precision)

        self.s1.collidable = False
        self.s2.collidable = False
        self.s3.collidable = False
        self.s4.collidable = False
        self.s5.collidable = False
        self.s6.collidable = False
        self.s7.collidable = False
        self.s8.collidable = False
        self.s9.collidable = False

        self.w.add(self.s1)
        self.w.add(self.s2)
        self.w.add(self.s3)
        self.w.add(self.s4)
        self.w.add(self.s5)
        self.w.add(self.s6)
        self.w.add(self.s7)
        self.w.add(self.s8)
        self.w.add(self.s9)

        self.listSensors = [self.s1, self.s2, self.s3, self.s4, self.s5, self.s6, self.s7,self.s8, self.s9]

        self.agent_in_range = []
        self.agentBuilding = []
        for agent in self.w.static_agents:
            if(isinstance(agent, RectangleBuilding) or isinstance(agent, CircleBuilding) or isinstance(agent, RingBuilding)):
                self.agentBuilding.append(agent)

        # Add Text

        self.textTest = TextAgent(Point(5*world_width, 5*world_height), "Speed " + str(round(self.c1.speed, 3)) + "\n" \
            "Travelled distance " + str(round(self.c1.distance_travelled, 3)) + "\n" \
            "Reward " + str(round(self.reward, 3)))
        self.textTest.collidable = False
        self.w.add(self.textTest)


        '''
            "Distance Front Sensor " + str(self.s1.dist_obstacle) + "\n" 
            + "Distance Diag Right Sensor " + str(self.s3.dist_obstacle) + "\n" 
            + "Distance Diag Left Sensor " + str(self.s2.dist_obstacle) + "\n"
            + "Distance Mid Left Sensor " + str(self.s4.dist_obstacle) + "\n"
            + "Distance Mid Right Sensor " + str(self.s5.dist_obstacle))
        '''

        # self.w.render(window_created) # This visualizes the world we just constructed.
        


    # def observe(self):
    #     #return state
    #      return tuple([self.s1.dist_obstacle, self.s2.dist_obstacle, self.s3.dist_obstacle])

    # def action(self, action): # Right positive
    #     if action == 0:
    #         self.c1.set_control(0.5, 0)
    #     elif action == 1:
    #         self.c1.set_control(0, 0)
    #     elif action == 2:
    #         self.c1.set_control(-2, 0)

        # anchor = Point(world_width/5, world_height/5)
        # self.info = Text(anchor, str(self.c1.speed))
        # self.w.add(self.info)

        # self.w.render()  # This visualizes the world we just constructed.


    def observe(self):
        #return state
        return tuple([sensor_state_obstacle(self.s1), sensor_state_obstacle(self.s2), \
            sensor_state_obstacle(self.s3), sensor_state_obstacle(self.s4), \
            sensor_state_obstacle(self.s5), sensor_state_obstacle(self.s6),\
            sensor_state_obstacle(self.s7), sensor_state_obstacle(self.s8),\
            sensor_state_obstacle(self.s9),\
            sensor_state_painting(self.s1), sensor_state_painting(self.s2), \
            sensor_state_painting(self.s3), sensor_state_painting(self.s4), \
            sensor_state_painting(self.s5), sensor_state_painting(self.s6),\
            sensor_state_painting(self.s7), sensor_state_painting(self.s8),\
            sensor_state_painting(self.s9),\
            speed_state(self.c1), \
            distance_state(self.c1, self.checkpoint, self.init_dist_to_cp)])

    def action(self, action):
        rotation = 0.3
        acceleration = 3

        if action == 0:
            self.c1.set_control(rotation, 0.5)
        elif action == 1:
            self.c1.set_control(-rotation, 0.5)
        elif action == 2:
            self.c1.set_control(0, acceleration)
        elif action == 3:
            self.c1.set_control(0, 0.5)

        # if action == 0:
        #     self.c1.set_control(rotation, 0.1)
        # elif action == 1:
        #     self.c1.set_control(rotation, 1)
        # elif action == 2:
        #     self.c1.set_control(rotation, -1)
        # elif action == 3:
        #     self.c1.set_control(-rotation, 0.1)
        # elif action == 4:
        #     self.c1.set_control(-rotation, 1)
        # elif action == 5:
        #     self.c1.set_control(-rotation, -1)
        # elif action == 6:
        #     self.c1.set_control(0, 0.1)
        # elif action == 7:
        #     self.c1.set_control(0, 1)
        # elif action == 8:
        #     self.c1.set_control(0, -1)



        self.w.tick() # This ticks the world for one time step (dt second)

        # Update sensors

        self.s1.car = self.c1
        self.s1.p1 = Point(self.s4.car.edge_centers[0][0]*self.ppm, self.s4.car.edge_centers[0][1]*self.ppm)
        self.s1.p2 = Point(self.s4.car.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s4.car.heading), 
            (self.ppm*self.s4.car.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.s4.car.heading))
        self.s1.list_points = list(getEquidistantPoints(self.s1.p1, self.s1.p2, self.precision))

         # Second sensor
        self.s2.car = self.c1
        self.s2.p1 = Point(self.s2.car.corners[0].x*self.ppm, self.s2.car.corners[0].y*self.ppm)
        self.s2.p2 = Point(self.s2.car.corners[0].x*self.ppm + self.ppm * self.range_sensors * np.cos(self.s2.car.heading + np.pi/4), 
            (self.ppm*self.s2.car.corners[0].y) + self.ppm * self.range_sensors * np.sin(self.s2.car.heading + np.pi/4))
        self.s2.list_points = list(getEquidistantPoints(self.s2.p1, self.s2.p2, self.precision))
        # Third sensor
        self.s3.car = self.c1
        self.s3.p1 = Point(self.s3.car.corners[3].x*self.ppm, self.s3.car.corners[3].y*self.ppm)
        self.s3.p2 = Point(self.s3.car.corners[3].x*self.ppm + self.ppm * self.range_sensors * np.cos(self.s1.car.heading - np.pi/4), 
            (self.ppm*self.s3.car.corners[3].y) + self.ppm * self.range_sensors * np.sin(self.s3.car.heading - np.pi/4))
        self.s3.list_points = list(getEquidistantPoints(self.s3.p1, self.s3.p2, self.precision))
        # Fourth sensor
        self.s4.car = self.c1
        self.s4.p1 = Point(self.s4.car.edge_centers[1][0]*self.ppm, self.s4.car.edge_centers[1][1]*self.ppm)
        self.s4.p2 = Point(self.s4.car.edge_centers[1][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s4.car.heading + np.pi/2), 
            (self.ppm*self.s4.car.edge_centers[1][1]) + self.ppm * self.range_sensors * np.sin(self.s4.car.heading + np.pi/2))
        self.s4.list_points = list(getEquidistantPoints(self.s4.p1, self.s4.p2, self.precision))
        # Fifth sensor
        self.s5.car = self.c1
        self.s5.p1 = Point(self.s5.car.edge_centers[3][0]*self.ppm, self.s5.car.edge_centers[3][1]*self.ppm)
        self.s5.p2 = Point(self.s5.car.edge_centers[3][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s5.car.heading - np.pi/2), 
            (self.ppm*self.s5.car.edge_centers[3][1]) + self.ppm * self.range_sensors * np.sin(self.s5.car.heading - np.pi/2))
        self.s5.list_points = list(getEquidistantPoints(self.s5.p1, self.s5.p2, self.precision))
        # Sixth sensor
        self.s6.car = self.c1
        self.s6.p1 = Point(self.s6.car.edge_centers[0][0]*self.ppm, self.s6.car.edge_centers[0][1]*self.ppm)
        self.s6.p2 = Point(self.s6.car.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s6.car.heading + np.pi/12), 
            (self.ppm*self.s6.car.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.s6.car.heading + np.pi/12))
        self.s6.list_points = list(getEquidistantPoints(self.s6.p1, self.s6.p2, self.precision))
        # Seventh sensor
        self.s7.car = self.c1
        self.s7.p1 = Point(self.s7.car.edge_centers[0][0]*self.ppm, self.s7.car.edge_centers[0][1]*self.ppm)
        self.s7.p2 = Point(self.s7.car.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s7.car.heading + np.pi/6), 
            (self.ppm*self.s7.car.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.s7.car.heading + np.pi/6))
        self.s7.list_points = list(getEquidistantPoints(self.s7.p1, self.s7.p2, self.precision))
        # Eighth sensor
        self.s8.car = self.c1
        self.s8.p1 = Point(self.s8.car.edge_centers[0][0]*self.ppm, self.s8.car.edge_centers[0][1]*self.ppm)
        self.s8.p2 = Point(self.s8.car.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s8.car.heading - np.pi/12), 
            (self.ppm*self.s8.car.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.s8.car.heading - np.pi/12))
        self.s8.list_points = list(getEquidistantPoints(self.s8.p1, self.s8.p2, self.precision))
        # Ninth sensor
        self.s9.car = self.c1
        self.s9.p1 = Point(self.s9.car.edge_centers[0][0]*self.ppm, self.s9.car.edge_centers[0][1]*self.ppm)
        self.s9.p2 = Point(self.s9.car.edge_centers[0][0]*self.ppm + self.ppm * self.range_sensors * np.cos(self.s9.car.heading - np.pi/6), 
            (self.ppm*self.s9.car.edge_centers[0][1]) + self.ppm * self.range_sensors * np.sin(self.s9.car.heading - np.pi/6))
        self.s9.list_points = list(getEquidistantPoints(self.s9.p1, self.s9.p2, self.precision))


        # Update all sensors' lines
        for sensor in self.listSensors:
            sensor.obj = []
            for i in range(sensor.precision):
                sensor.obj.append(Line(Point(sensor.list_points[i][0], sensor.list_points[i][1]),Point(sensor.list_points[i+1][0], sensor.list_points[i+1][1])))

        self.s1.collidable = False
        self.s2.collidable = False
        self.s3.collidable = False
        self.s4.collidable = False
        self.s5.collidable = False
        self.s6.collidable = False
        self.s7.collidable = False
        self.s8.collidable = False
        self.s9.collidable = False
        
        # Sensors detection parallelization
        # threading.Thread(target = self.sensor_detection_obstacle).start()
        # threading.Thread(target = self.sensor_detection_painting).start()
        self.sensor_detection_obstacle()
        self.sensor_detection_painting()

        self.car_in_checkpoint()

        # Update Text

        self.textTest.text = "Speed " + str(round(self.c1.speed, 3)) + "\n" \
        + "Travelled distance " + str(round(self.c1.distance_travelled, 3)) + "\n" \
        + "Reward " + str(round(self.reward, 3)) + "\n" \
        + "Distance Front " + str(self.s1.dist_obstacle) + "\n" \
        + "Distance Diag Left " + str(self.s2.dist_obstacle) + "\n" \
        + "Distance Diag Right " + str(self.s3.dist_obstacle) + "\n" \
        + "Distance Mid Left " + str(self.s4.dist_obstacle) + "\n" \
        + "Distance Mid Right " + str(self.s5.dist_obstacle) + "\n" \
        + "Distance Front Left " + str(self.s6.dist_obstacle) + "\n" \
        + "Distance Front Left 2 " + str(self.s7.dist_obstacle) + "\n" \
        + "Distance Front Right " + str(self.s8.dist_obstacle) + "\n" \
        + "Distance Front Right 2 " + str(self.s9.dist_obstacle)
        

        self.textTest.collidable = False
        
        # For evaluation
        self.c1.distance_travelled += self.c1.speed*dt

        self.w.render(self.window_created)

        # time.sleep(0.1) # Let's watch it 4x



        if self.w.collision_exists(): # We can check if there is any collision at all.
            pass
            # print('Collision exists somewhere...')

        if self.w.car_cross_line():
            pass
            # print("CAR CROSSES LINE")

        self.agent_in_range = []
        for agent in (self.w.static_agents + self.w.dynamic_agents):
            if not (isinstance(agent, TextAgent) or \
                isinstance(agent, CheckPointAgent) or \
                isinstance(agent, Sensors) or \
                isinstance(agent, RingBuilding) or \
                isinstance(agent, CircleBuilding) or \
                isinstance(agent, RectangleBuilding) or \
                # isinstance(agent, CheckPointAgent) or\
                (isinstance(agent, Car) and agent.name == "agent")):
                if self.c1.distanceTo(agent) < self.range_sensors+1:
                    self.agent_in_range.append(agent)
        # print(len(self.agent_in_range))

    def sensor_detection_obstacle(self):
        # Sensors detection
        for sensor in self.listSensors:
            sensor.dist_obstacle = self.range_sensors
        for agent in (self.agent_in_range + self.agentBuilding):
            if not isinstance(agent, Painting):
                for sensor in self.listSensors:
                    for mRange in range(sensor.precision):
                        if (Line( sensor.obj[mRange].p1/self.ppm, sensor.obj[mRange].p2/self.ppm).intersectsWith(agent.obj)):      
                            sensor.dist_obstacle = mRange * (self.range_sensors / sensor.precision)
                            break

    def sensor_detection_painting(self):
        for sensor in self.listSensors:
            sensor.closest_painting = self.range_sensors
        for agent in self.agent_in_range:
            if isinstance(agent, Painting):
                for sensor in self.listSensors:
                    # if(sensor.name == "Mid_Right_Sensor"):
                    #     print("FIRST MRS: ", sensor.obj[0].p1/self.ppm, sensor.obj[0].p2/self.ppm)
                    #     print("LAST MRS: ", sensor.obj[9].p1/self.ppm, sensor.obj[9].p2/self.ppm)
                    # if(sensor.name == "Mid_Left_Sensor"):
                    #     print("FIRST MLS: ", sensor.obj[0].p1/self.ppm, sensor.obj[0].p2/self.ppm)
                    #     print("LAST MLS: ", sensor.obj[9].p1/self.ppm, sensor.obj[9].p2/self.ppm)

                    for mRange in range(sensor.precision):
                        if (Line(sensor.obj[mRange].p1/self.ppm, sensor.obj[mRange].p2/self.ppm).intersectsWith(agent.obj)):
                            sensor.closest_painting = mRange * (self.range_sensors / sensor.precision)
                            # print(sensor.name, "detects", agent, "range", mRange)
                            break

    def car_in_checkpoint(self):
        if (abs(self.c1.center.x - self.checkpoint.center.x)-10 <= 0) and \
                (abs(self.c1.center.y - self.checkpoint.center.y)-10 <= 0):

            r = (50-10) * sqrt(random.uniform(0, 1))
            theta = random.uniform(0, 1) * 2 * np.pi
            x = world_height/2 + r * np.cos(theta)
            y = world_width/2 + r * np.sin(theta)

            self.checkpoint.center.x = x
            self.checkpoint.center.y = y

            self.init_dist_to_cp = self.c1.distanceTo(self.checkpoint)



    def car_outside_map(self):
        return self.c1.center.x>120 or self.c1.center.y>120 or self.c1.center.y<0 or self.c1.center.x<0

    def evaluate(self):
        reward = 0
        if self.w.collision_exists() or self.car_outside_map():  # We can check if there is any collision at all.
            reward += -10000
        else:
            reward += self.c1.distance_travelled*10
        if self.c1.speed < speed_minimum or self.c1.speed > speed_limit:
            reward += -100
        if self.c1.speed > 3:
            reward += 10
        if self.w.car_cross_line(): # car crosses road marking
            reward -= 1000
        reward += -100*distance_state(self.c1, self.checkpoint, self.init_dist_to_cp)
        self.reward = reward
        return reward

    def is_done(self):
        if self.w.collision_exists() or self.car_outside_map():
            self.w.erase_car()
            # self.carlo.w.visualizer.close()
            # self.w.visualizer.close()
            return True
        # if self.c1.speed == 0:
        #     return True
        return False

    def view(self, window_created: bool):
        self.w.tick() # This ticks the world for one time step (dt second)
        self.w.render(window_created)
        if self.w.collision_exists():

            self.w.erase_car()

        # time.sleep(5) # Let's watch it 4x

    def circular_world(self):
        # To create a circular road, we will add a CircleBuilding and then a RingBuilding around it
        self.cb = CircleBuilding(Point(world_width/2, world_height/2), inner_building_radius, 'gray80')
        self.w.add(self.cb)
        self.rb = RingBuilding(Point(world_width/2, world_height/2), inner_building_radius + num_lanes * lane_width + (num_lanes - 1) * lane_marker_width, 1+np.sqrt((world_width/2)**2 + (world_height/2)**2), 'gray80')
        self.w.add(self.rb)
        

        # Let's also add some lane markers on the ground. This is just decorative. Because, why not.
        for lane_no in range(num_lanes - 1):
            self.lane_markers_radius = inner_building_radius + (lane_no + 1) * lane_width + (lane_no + 0.5) * lane_marker_width
            self.lane_marker_height = np.sqrt(2*(self.lane_markers_radius**2)*(1-np.cos((2*np.pi)/(2*num_of_lane_markers)))) # approximate the circle with a polygon and then use cosine theorem
            for theta in np.arange(0, 2*np.pi, 2*np.pi / num_of_lane_markers):
                self.dx = self.lane_markers_radius * np.cos(theta)
                self.dy = self.lane_markers_radius * np.sin(theta)
                self.w.add(Painting(Point(world_width/2 + self.dx, world_height/2 + self.dy), Point(lane_marker_width, self.lane_marker_height), 'white', heading = theta))


        # A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
        self.c1 = Car(Point(91.75, 60), np.pi/2)
        self.c1.max_speed = 30.0  # let's say the maximum is 30 m/s (108 km/h)
        self.c1.velocity = Point(0, 3.0)
        self.c1.name = "agent"
        self.w.add(self.c1)
        self.ppm = 6

    def intersect_world(self):
        # Intersection map
        # Top right
        self.w.add(Painting(Point(81.5, 111.5), Point(97, 27), 'gray80'))  # We build a sidewalk.
        self.w.add(RectangleBuilding(Point(82.5, 112.5), Point(95, 25)))  # The RectangleBuilding is then on top of the sidewalk, with some margin.

        # Let's repeat this for 4 different RectangleBuildings.
        # Top Left
        self.w.add(Painting(Point(8.5, 111.5), Point(17, 27), 'gray80'))
        self.w.add(RectangleBuilding(Point(7.5, 112.5), Point(15, 25)))
        # Bottom Left
        self.w.add(Painting(Point(8.5, 41), Point(17, 82), 'gray80'))
        self.w.add(RectangleBuilding(Point(7.5, 40), Point(15, 80)))
        # Bottom Right
        self.w.add(Painting(Point(81.5, 41), Point(97, 82), 'gray80'))
        self.w.add(RectangleBuilding(Point(82.5, 40), Point(95, 80)))


        # # Let's also add some zebra crossings.
        # for i in range(0, 15, 2):
        #     self.w.add(Painting(Point(18+i, 81), Point(0.5, 2), 'white'))


        self.checkpoint = CheckPointAgent(Point(78, 85), np.pi, color="orange")
        self.w.add(self.checkpoint)


        # Let's also add some lane markers on the ground.
        for i in range (0, 81, 5):
            self.w.add(Painting(Point(25, i), Point(0.5, 3), 'white'))
            # self.w.add(Painting(Point(25, 100 + i), Point(0.5, 3), 'white'))
            self.w.add(Painting(Point(i + 35, 90), Point(3, 0.5), 'white'))
            # self.w.add(Painting(Point(i - 65, 90), Point(3, 0.5), 'white'))

        # A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
        self.c1 = Car(Point(30, 35), np.pi/2)
        self.c1.max_speed = 30.0  # let's say the maximum is 30 m/s (108 km/h)
        self.c1.velocity = Point(0, 3.0)
        self.c1.name = "agent"
        self.w.add(self.c1)
        self.ppm = 6

        self.c2 = Car(Point(118, 94), np.pi, 'blue')
        self.c2.velocity = Point(3.0, 0)  # We can also specify an initial velocity just like this.
        self.w.add(self.c2)

        # Pedestrian is almost the same as Car. It is a "circle" object rather than a rectangle.
        self.p1 = Pedestrian(Point(40, 81), np.pi)
        self.p1.max_speed = 10.0  # We can specify min_speed and max_speed of a Pedestrian (and of a Car). This is 10 m/s, almost Usain Bolt.
        self.p1.velocity = Point(-1, 0)
        self.w.add(self.p1)

        self.init_dist_to_cp = self.c1.distanceTo(self.checkpoint)

        # Pedestrian
        self.p1.velocity = Point(-1, 0)

    def random_checkpoint_world(self):
        # Map
        self.rb = RingBuilding(Point(world_width /2, world_height /2),
                               50,
                               1 + np.sqrt((world_width) ** 2 + (world_height) ** 2), 'gray80')

        # Car
        self.c1 = Car(Point(world_width/2, world_height/2), np.pi / 2)
        self.c1.max_speed = 30.0  # let's say the maximum is 30 m/s (108 km/h)
        self.c1.velocity = Point(0, 3.0)
        self.c1.name = "agent"
        self.ppm = 6

        # Checkpoint
        r = 50 * sqrt(random.uniform(0, 1))
        theta = random.uniform(0, 1) * 2 * np.pi
        x = world_height / 2 + r * np.cos(theta)
        y = world_width / 2 + r * np.sin(theta)
        self.checkpoint = CheckPointAgent(Point(x, y), 10, color="orange")
        # self.checkpoint = CheckPointAgent(Point(world_height/2, world_width/2+60), 10, color="orange")

        self.init_dist_to_cp = self.c1.distanceTo(self.checkpoint)

        # Add elements to map (order matters for visuals (only for non-movable objects)
        self.w.add(self.rb)
        self.w.add(self.checkpoint)
        self.w.add(self.c1)
