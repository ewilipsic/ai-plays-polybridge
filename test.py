import pygame
import pymunk
import random
import time

class Bridge:
    def __init__(self, nodes, edges, static_nodes):
        """
        space: pymunk.Space - the physics space
        nodes: dict - {node_id: (x, y)} coordinates of nodes
        edges: list - {(node_a, node_b): mass} edges by node ids with mass
        static_nodes: list or set - node ids that are static with respect to world
        """
        self.space = pymunk.Space()
        self.space.gravity = (0, 900)
        self.space.damping = 0.5
        self.space.iterations = 500

        self.nodes = nodes
        self.edges = edges
        self.static_nodes = static_nodes

        self.bodies = {}
        self.shapes = []
        self.joints = {}

        adj = {} # node_id: [node_id1, ...]

        for (a, b), mass in self.edges.items():
            if a > b:
                a, b = b, a

            if a not in adj.keys():
                adj[a] = []
            if b not in adj.keys():
                adj[b] = []
            adj[a].append(b)
            adj[b].append(a)

            a_pos = self.nodes[a]
            b_pos = self.nodes[b]

            body = pymunk.Body()
            body.position = (a_pos + b_pos) / 2
            self.space.add(body)
            self.bodies[a, b] = body

            segment = pymunk.Segment(body, a_pos - body.position, b_pos - body.position, radius=5)
            segment.mass = mass
            self.space.add(segment)
            self.shapes.append(segment)
        
        for a, nbhs in adj.items():
            a_pos = self.nodes[a]

            for i in range(len(nbhs)):
                for j in range(i + 1, len(nbhs)):
                    a1 = a
                    a2 = a
                    b = nbhs[i]
                    c = nbhs[j]

                    if a1 > b:
                        a1, b = b, a1
                    if a2 > c:
                        a2, c = c, a2

                    b_body = self.bodies[a1, b]
                    c_body = self.bodies[a2, c]
                    
                    joint = pymunk.PivotJoint(b_body, c_body, a_pos)
                    joint.collide_bodies = False
                    self.space.add(joint)
                    self.joints[a] = joint

        for a, b in self.edges:
            if a in self.static_nodes:
                stat = a
            elif b in self.static_nodes:
                stat = b
            else:
                continue

            if a > b:
                a, b = b, a

            body = pymunk.Body(body_type=pymunk.Body.STATIC)
            body.position = self.nodes[stat]
            self.space.add(body)

            joint = pymunk.PivotJoint(body, self.bodies[a, b], body.position)
            joint.collide_bodies = False
            self.space.add(joint)
            self.joints[a] = joint
            
def draw_bridge(screen, bridge):
    screen.fill((255, 255, 255))

    for segment in bridge.shapes:
        body = segment.body
        p1 = body.position + segment.a.rotated(body.angle)
        p2 = body.position + segment.b.rotated(body.angle)
        pygame.draw.line(screen, (0, 0, 0), p1, p2, 5)
        pygame.draw.circle(screen, (255, 0, 0), p1, 7.5)
        pygame.draw.circle(screen, (255, 0, 0), p2, 7.5)

def cross_nodes(a_nodes, b_nodes):
    nodes = {}
    for n, pos1 in a_nodes.items():
        min_dist = float('inf')
        close_pos2 = None
        for pos2 in b_nodes.values():
            dist = (pos2 - pos1).length
            if dist < min_dist:
                close_pos2 = pos2
                min_dist = dist

        if random.random() < 0.5:
            nodes[n] = close_pos2
        else:
            nodes[n] = pos1

    return nodes

def main():
    pygame.init()

    width, height = 600, 400
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Pymunk Bridge Graph with Pygame")

    clock = pygame.time.Clock()

    a_nodes = {
        1: pymunk.Vec2d(100, 200),
        2: pymunk.Vec2d(200, 200),
        3: pymunk.Vec2d(300, 200),
        4: pymunk.Vec2d(400, 200),
        5: pymunk.Vec2d(150, 300),
        6: pymunk.Vec2d(200, 250),
    }

    b_nodes = {
        1: pymunk.Vec2d(120, 210),
        2: pymunk.Vec2d(230, 220),
        3: pymunk.Vec2d(340, 210),
        4: pymunk.Vec2d(420, 210),
        5: pymunk.Vec2d(150, 320),
        6: pymunk.Vec2d(250, 250),
    }

    edges = {
        (1, 2): 10,
        (2, 3): 10,
        (3, 4): 10,
        (1, 5): 10,
        (5, 6): 10,
    }

    static_nodes = {1, 3}

    random.seed(time.time())
    child = Bridge(cross_nodes(a_nodes, b_nodes), edges, static_nodes)

    prev_time = int(time.time())

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        child.space.step(1 / 60.0)

        draw_bridge(screen, child)

        cur_time = int(time.time())
        if cur_time > prev_time:
            child = Bridge(cross_nodes(a_nodes, b_nodes), edges, static_nodes)
            prev_time = cur_time

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
