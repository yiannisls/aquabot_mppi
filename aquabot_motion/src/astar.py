import pylab as plt
import numpy as np
from importlib import reload
import heapq
from random import randint

import bitangents as bt

reload(bt)


class Astar:

    def __init__(self):
        self.no_go_zones = []

    def add_obstacle(self, x, y, rad):

        for obs in self.no_go_zones:
            if obs.x == x and obs.y == y:
                return

        self.no_go_zones.append(bt.Obstacle(x, y, rad))

    def init_graph(self):
        self.nodes, self.edges = bt.generate_graph(self.no_go_zones)
        self.N = len(self.nodes)

    def register_edge(self, i1, i2):
        self.edges[i1,i2] = self.edges[i2,i1] = self.nodes[i1].distance(self.nodes[i2])

    def project(self, node):

        while True:
            free = True
            for obs in self.no_go_zones:
                if obs.contains(node.x, node.y):
                    free = False
                    # get out of here
                    angle = np.arctan2(node.y-obs.y, node.x-obs.x)
                    node = bt.Point(obs.x + (obs.rad+0.1)*np.cos(angle),
                                    obs.y + (obs.rad+0.1)*np.sin(angle))
            if free:
                return node

    def set_task(self, xs, ys, xg, yg, discretize = False):

        N = self.N

        # reset base nodes
        self.nodes = self.nodes[:N]
        self.edges = self.edges[:N,:N]

        start = bt.Point(xs,ys)
        goal = bt.Point(xg,yg)

        new_nodes, ns = bt.generate_tangents(self.no_go_zones,
                                              self.project(start),
                                              self.project(goal))

        self.nodes += new_nodes

        added = len(new_nodes)
        # add edges
        self.edges = np.hstack((np.vstack((self.edges, -1*np.ones((added, N)))),
                                -1*np.ones((N+added, added))))
        # start and goal
        if ns == 0:
            self.register_edge(N,N+1)
            path = [start,goal]
        else:

            # start to tangents
            for i in range(N+2,N+2+ns):
                self.register_edge(N, i)
            for i in range(N+2+ns, len(self.nodes)):
                self.register_edge(N+1, i)

            # add remaining arcs
            for i,p1 in enumerate(self.nodes[N+2:]):
                i1 = i + N+2
                for i2, p2 in enumerate(self.nodes):
                    if i1 == i2 or p1.obs != p2.obs:
                        continue
                    self.register_edge(i1, i2)

            path = self.solve()

            if path is None:
                return None
            path = [start] + path + [goal]

        if not discretize:
            return path

        XY = np.hstack([path[i].discretize(path[i+1]) for i in range(len(path)-1)]).T
        return [bt.Point(p[0],p[1]) for p in XY]

    def solve(self):

        goal = self.nodes[self.N+1]
        nodes = self.nodes

        class Node:
            def __init__(self, idx, parent = None, dg = 0):
                self.idx = idx
                self.parent = parent
                if parent is None:
                    self.g = 0
                else:
                    self.g = parent.g + dg

            def __lt__(self, other):
                return self.h() < other.h()

            def node(self):
                return nodes[self.idx]

            def h(self):
                return self.g + self.node().distance(goal)

        frontier = []

        def addNode(idx, parent = None, dg = 0):
            node = Node(idx, parent, dg)
            heapq.heappush(frontier, (node.h(), node))

        addNode(self.N)

        closed = []
        max_iter = 500
        it = 0
        while frontier:
            it += 1

            if it == max_iter:
                return None

            best = heapq.heappop(frontier)[1]

            if best.idx == self.N+1:
                break
            closed.append(best.idx)

            for idx,dist in enumerate(self.edges[best.idx]):
                if dist > 0 and idx not in closed:
                    addNode(idx, best, dist)

        # reverse
        path = []
        while True:
            path.insert(0, nodes[best.idx])
            if best.parent is None:
                break
            best = best.parent

        return path

    def plot(self, edges = False):

        plt.close('all')

        fig, ax = plt.subplots()

        # Plot each obstacle
        round = plt.linspace(-np.pi, np.pi, 200)
        cr = plt.cos(round)
        sr = plt.sin(round)
        for obstacle in self.no_go_zones:
            plt.plot(obstacle.x + obstacle.rad * cr, obstacle.y + obstacle.rad * sr, 'k--')

        # Plot edges
        def color(i1, i2):
            old = i1 < self.N and i2 < self.N
            same = self.nodes[i1].obs == self.nodes[i2].obs
            if old:
                return 'b' if same else 'r'
            return 'C0' if same else 'C1'

        for i, p1 in enumerate(self.nodes[:-1]):
            for j in range(i+1, len(self.nodes)):
                p2 = self.nodes[j]
                if self.edges[i,j] > 0:
                    XY = p1.discretize(p2)
                    plt.plot(XY[0], XY[1], color(i,j))


        # Set the limits of the plot
        ax.set_xlim(-250,250)
        ax.set_aspect('equal', 'box')

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    astar = Astar()

    obstacles = (
        (120, -50, 35),  # aquabot_lighthouse_island
    (-152, -6, 55),  # aquabot_island_1
    (110, 135, 50),  # aquabot_island_2
    (12, -102, 30),  # aquabot_rock_island_0
    (92, 170, 30),  # aquabot_rock_island_1
    (-92, 176, 40),  # aquabot_rock_0
    (-40, 220, 32),  # aquabot_rock_1
    (-44, -95, 32),  # aquabot_rock_2
    (-30, -150, 32),  # aquabot_rock_3
    )

    for x,y,rad in obstacles:
        astar.add_obstacle(x,y,rad)

    astar.init_graph()

    astar.plot()
    line, = plt.plot([], [], 'g', linewidth = 2)
    nodes, = plt.plot([], [], 'C1D')


    def rand():
        return randint(-200,200)


    while True:

        xs,ys,xg,yg =  -88 , -92 , 92 , 52 # rand(), rand(), rand(), rand()
        xs,ys,xg,yg = rand(), rand(), rand(), rand()

        path = astar.set_task(xs,ys,xg,yg,True)
        #XY = np.hstack((path[i].discretize(path[i+1]) for i in range(len(path)-1)))
        #line.set_data(XY[0],XY[1])

        XY = np.hstack(([[p.x],[p.y]] for p in path))
        line.set_data(XY[0],XY[1])

        plt.draw()
        plt.pause(1)

    #for i,p1 in enumerate(path[:-1]):
        #p2 = path[i+1]
        #XY = p1.discretize(p2)
        #plt.plot(XY[0], XY[1], 'g', linewidth = 2)

    #for p in path:
        #plt.plot([p.x],[p.y],'C1D')
