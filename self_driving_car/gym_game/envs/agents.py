from gym_game.envs.entities import RectangleEntity, CircleEntity, RingEntity, SensorEntity, TextEntity
from gym_game.envs.geometry import Point

import numpy

def getEquidistantPoints(p1: Point, p2: Point, parts: int):
    return zip(numpy.linspace(p1.x, p2.x, parts+1),
               numpy.linspace(p1.y, p2.y, parts+1))

# For colors, we use tkinter colors. See http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter

class Car(RectangleEntity):
    def __init__(self, center: Point, heading: float, color: str = 'red'):
        size = Point(4., 2.)
        movable = True
        friction = 0.0
        super(Car, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = True
        self.inertia = True
        self.distance_travelled = 0
        self.name = "npc"
        
class Pedestrian(CircleEntity):
    def __init__(self, center: Point, heading: float, color: str = 'LightSalmon3'): # after careful consideration, I decided my color is the same as a salmon, so here we go.
        radius = 1
        movable = True
        friction = 0.2
        super(Pedestrian, self).__init__(center, heading, radius, movable, friction)
        self.color = color
        self.collidable = True
        self.inertia = True
        
class RectangleBuilding(RectangleEntity):
    def __init__(self, center: Point, size: Point, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(RectangleBuilding, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = True
        
class CircleBuilding(CircleEntity):
    def __init__(self, center: Point, radius: float, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(CircleBuilding, self).__init__(center, heading, radius, movable, friction)
        self.color = color
        self.collidable = True

class RingBuilding(RingEntity):
    def __init__(self, center: Point, inner_radius: float, outer_radius: float, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(RingBuilding, self).__init__(center, heading, inner_radius, outer_radius, movable, friction)
        self.color = color
        self.collidable = True

class Painting(RectangleEntity):
    def __init__(self, center: Point, size: Point, color: str = 'gray26', heading: float = 0.):
        movable = False
        friction = 0.
        super(Painting, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = False

class Sensors(SensorEntity):
    def __init__(self, p1: Point, p2: Point, range_sensors: int, name: str, car: Car, precision: int, color: str = 'green'):
        self.collidable = False         
        self.p1 = p1
        self.p2 = p2
        self.range_sensors = range_sensors
        movable = True
        self.inertia = False
        self.name = name
        self.car = car
        self.precision = precision # sensor's resolution
        self.list_points = list(getEquidistantPoints(p1, p2, precision))
        self.dist_obstacle = range_sensors
        self.closest_painting = range_sensors
        super(Sensors, self).__init__(p1, p2, range_sensors, car)

class TextAgent(TextEntity):
    def __init__(self, anchor: Point, text: str, color: str = "black"):
        self.movable = True
        self.color = "black"
        self.text = text
        self.anchor = anchor
        self.collidable = False
        self.inertia = False
        self.color = color
        super(TextAgent, self).__init__(anchor, text)
        

class CheckPointAgent(CircleEntity):
    def __init__(self, center: Point, radius: float, color: str = 'light blue'):
        heading = 0.
        movable = True
        friction = 0.
        super(CheckPointAgent, self).__init__(center, heading, radius, movable, friction)
        self.color = color
        self.inertia = False
        self.collidable = False
