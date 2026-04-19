from math import atan2, acos, cos, sin, sqrt, pi
import numpy as np

eps = 1e-3


class Obstacle:
    def __init__(self, x, y, rad):
        self.x = x
        self.y = y
        self.rad = rad
        self.collisions = []

    def collides(self, other):
        return self.distance(other) < self.rad + other.rad + eps
    
    def collides_with_segment(self, p1, p2):
        # Vector from p1 to p2
        dx = p2.x - p1.x
        dy = p2.y - p1.y

        # Vector from p1 to the center of the obstacle
        fx = p1.x - self.x
        fy = p1.y - self.y

        a = dx * dx + dy * dy

        if a == 0:
            return False
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - self.rad * self.rad

        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            # No intersection
            return False

        # Check if the intersection points are within the segment
        discriminant = sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def pixels(self, dim):
        return int(self.x+dim/2), int(dim/2-self.y)

    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def contains(self, x, y):
        return (self.x-x)**2 + (self.y-y)**2 < self.rad**2 + eps

    def arc(self, steps):
        return np.array([[self.x, self.y]]).T + self.rad*np.vstack((np.cos(steps), np.sin(steps)))


class Point:

    obstacles = []

    def __init__(self, x, y):
        self.lefty = None
        if isinstance(x, Obstacle):
            self.x = x.x + x.rad * cos(y)
            self.y = x.y + x.rad * sin(y)
            self.obs = x
            self.angle = y % (2*np.pi)
        else:
            self.x = x
            self.y = y
            self.obs = self.angle = None

    def distance(self, other):
        if self.obs is None or other.obs is None or self.obs != other.obs:
            return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
        # on same circle / need to check collision
        angle = self.arc(other)[1]
        if angle is None:
            return -1
        return self.obs.rad * abs(angle)

    def discretize(self, other):
        '''
        only called with valid pair
        '''
        if self.obs is None or other.obs is None or self.obs != other.obs:
            dxy = np.array([[other.x - self.x, other.y - self.y]]).T
            d = self.distance(other)
            steps = np.linspace(0, 1, int(d)).reshape(1,-1)
            return np.array([[self.x, self.y]]).T + np.dot(dxy,steps)
        return self.arc(other)[0]

    def arc(self, other):

        if self.lefty == other.lefty:
            return None, None

        if self.lefty:  # or not other.lefty:
            arc, angle = other.arc(self)
            if angle is None:
                return None, None
            return arc[:,::-1], -angle
        angle = other.angle - self.angle
        if angle < 0:
            angle += 2*pi
        steps = self.angle + np.linspace(0, angle, 4*int(angle*self.obs.rad))
        points = np.array([[self.obs.x, self.obs.y]]).T + self.obs.rad*np.vstack((np.cos(steps), np.sin(steps)))

        points = self.obs.arc(steps)

        for p in points.T:
            if any([obs.contains(p[0], p[1]) for obs in self.obs.collisions]):
                return None, None

        return points, angle

    def orient(self, other):
        # sets lefty param on p depending on other
        dx = other.x - self.x
        dy = other.y - self.y
        px = self.obs.x - self.x
        py = self.obs.y - self.y

        self.lefty = dx*py - dy*px > 0
        return self


def segment_collides_with_obstacles(p1, p2, obstacles, prune = False):
    for obstacle in obstacles:
        if prune and obstacle in (p1.obs, p2.obs):
            continue
        if obstacle.collides_with_segment(p1, p2):
            return True
    return False


def generate_bitangents(obstacles):

    # clean obstacles first
    for obs in obstacles:
        obs.collisions = []
    obstacles.sort(key=lambda o: o.rad)
    Point.obstacles = obstacles
    bitangents = []
    nodes = []
    for i,o1 in enumerate(obstacles[:-1]):
        for o2 in obstacles[i+1:]:
            dx = o2.x - o1.x
            dy = o2.y - o1.y
            d = sqrt(dx*dx + dy*dy)
            offset = atan2(dy, dx)
            pairs = []

            # external tangent
            if d > abs(o1.rad - o2.rad):
                angle = acos((o1.rad - o2.rad) / d)
                for s in [1, -1]:
                    pairs.append((Point(o1, offset + s*angle), Point(o2, offset + s*angle)))

            # internal
            if d > o1.rad + o2.rad:
                angle = acos((o1.rad + o2.rad) / d)
                for s in [1, -1]:
                    pairs.append((Point(o1, offset + s*angle), Point(o2, offset + s*(angle + pi))))
            else:
                o1.collisions.append(o2)
                o2.collisions.append(o1)

            for p1, p2 in pairs:
                if not segment_collides_with_obstacles(p1, p2, obstacles, True):
                    bitangents.append((p1.orient(p2), p2.orient(p1)))
                    nodes += [p1,p2]
    return bitangents, nodes


def generate_graph(obstacles):

    bitangents, nodes = generate_bitangents(obstacles)
    N = len(nodes)
    edges = -1*np.ones((N,N))
    # raw edges
    for i, p1 in enumerate(nodes[:-1]):

        if i % 2 == 0:
            edges[i,i+1] = edges[i+1,i] = nodes[i].distance(nodes[i+1])

        for j in range(i+2-(i % 2), N):
            if p1.obs == nodes[j].obs:
                edges[i,j] = edges[j,i] = nodes[i].distance(nodes[j])
    return nodes, edges


def generate_tangents(obstacles, start, goal):

    # if start can see goal, just use these ones
    if not segment_collides_with_obstacles(start, goal, obstacles):
        return [start, goal], 0

    nodes = []
    ns = None

    for node in (start, goal):

        for obs in obstacles:
            d = obs.distance(node)

            if d < obs.rad:
                continue
            angle = np.arccos(obs.rad/d)
            offset = np.arctan2(node.y-obs.y, node.x-obs.x)
            for s in [-1,1]:
                dst = Point(obs, offset + s*angle)
                if not segment_collides_with_obstacles(node, dst, obstacles, True):
                    nodes.append(dst.orient(node))

        if ns is None:
            ns = len(nodes)

    return [start, goal] + nodes, ns

