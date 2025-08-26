import pygame
import pymunk
import pymunk.pygame_util
import random
import time

CATEGORY_1 = 0b001
CATEGORY_2 = 0b010
CATEGORY_3 = 0b100

# Create shapes with appropriate filters
# Object 1: Can collide with categories 2 and 3
road = pymunk.ShapeFilter(
    categories=CATEGORY_1,
    mask=CATEGORY_2 | CATEGORY_3
)

# Object 2: Can collide with category 1 only
ball = pymunk.ShapeFilter(
    categories=CATEGORY_2,           # This object is type 2
    mask=CATEGORY_1                  # Can collide with type 1 only
)

# Object 3: Can collide with category 1 only
support = pymunk.ShapeFilter(
    categories=CATEGORY_3,           # This object is type 3
    mask=CATEGORY_1                  # Can collide with type 1 only
)

class Bridge:
    def __init__(self, nodes, static_nodes, edges):
        """
        nodes: dict - {node_id: (x, y)} coordinates of nodes
        edges: list - {(a_node_id, b_node_id): mass} edges by node ids with mass
        static_nodes: list or set - node ids that are static with respect to world
        """
        self.space = pymunk.Space()
        self.space.gravity = (0, 900)
        self.space.damping = 0.5
        self.space.iterations = 500

        self.nodes = nodes
        self.static_nodes = static_nodes
        self.edges = edges

        self.adj = {} # a_node_id: [b_node_id, ...]
        self.seg_bodies = {} # (a_node_id, b_node_id): body
        self.joints = []

        for (a, b), (mass, is_road) in self.edges.items():
            if a > b:
                a, b = b, a

            if a not in self.adj.keys():
                self.adj[a] = []
            if b not in self.adj.keys():
                self.adj[b] = []
            self.adj[a].append(b)
            self.adj[b].append(a)

            a_pos = self.nodes[a]
            b_pos = self.nodes[b]

            body = pymunk.Body()
            body.position = (a_pos + b_pos) / 2
            self.space.add(body)
            self.seg_bodies[a, b] = body

            segment = pymunk.Segment(body, a_pos - body.position, b_pos - body.position, radius=5)
            segment.mass = mass
            segment.filter = road if is_road else support
            self.space.add(segment)

        for a, nbhs in self.adj.items():
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

                    b_body = self.seg_bodies[a1, b]
                    c_body = self.seg_bodies[a2, c]

                    joint = pymunk.PivotJoint(b_body, c_body, a_pos)
                    joint.collide_bodies = False
                    self.space.add(joint)
                    self.joints.append(joint)

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

            joint = pymunk.PivotJoint(body, self.seg_bodies[a, b], body.position)
            joint.collide_bodies = False
            self.space.add(joint)
            self.joints.append(joint)

def mutate(bridge):
    nodes = bridge.nodes
    edges = bridge.edges

    return Bridge(nodes, bridge.static_nodes, edges)

def cross_nodes(a_nodes, b_nodes, a_static_nodes):
    nodes = {}
    for n, pos1 in a_nodes.items():
        if n in a_static_nodes or random.random() > 0.5: # prob of gene cross
            nodes[n] = pos1
            continue

        min_dist = float('inf')
        close_pos2 = None
        for pos2 in b_nodes.values():
            dist = (pos2 - pos1).length
            if dist < min_dist:
                close_pos2 = pos2
                min_dist = dist

        nodes[n] = close_pos2

    return nodes

def constant_horizontal_velocity(body, gravity, damping, dt):
    """Custom velocity function to maintain constant horizontal speed"""
    target_horizontal_velocity = 100
    pymunk.Body.update_velocity(body, gravity, damping, dt)
    body.velocity = (target_horizontal_velocity, body.velocity.y)

def comp_max_tension(bridge, start_pos, end_x, screen=None, draw_options=None, clock=None):
    tps = 60
    dt = 1 / tps

    body = pymunk.Body(mass=10, moment=10)
    body.position = start_pos
    body.velocity_func = constant_horizontal_velocity
    shape = pymunk.Circle(body, 20)
    shape.filter = ball
    bridge.space.add(body, shape)

    max_tension = 0

    while body.position.x <= end_x:
        bridge.space.step(dt)

        for joint in bridge.joints:
            tension = joint.impulse * tps
            max_tension = max(max_tension, tension)

        if screen is None or draw_options is None:
            continue

        screen.fill("white")
        bridge.space.debug_draw(draw_options)
        pygame.display.update()

        if clock is None:
            continue

        clock.tick(tps)

    return max_tension

def main():
    pygame.init()

    width, height = 600, 400
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Pymunk Bridge Graph with Pygame")

    draw_options = pymunk.pygame_util.DrawOptions(screen)
    clock = pygame.time.Clock()

    random.seed(time.time())

    a_nodes = {
        1: pymunk.Vec2d(100, 200),
        2: pymunk.Vec2d(200, 200),
        3: pymunk.Vec2d(300, 200),
        4: pymunk.Vec2d(400, 200),
        5: pymunk.Vec2d(150, 100),
        6: pymunk.Vec2d(250, 100),
        7: pymunk.Vec2d(350, 100),   
    }

    a_static_nodes = {1, 7}
    
    a_edges = {
        (1, 2): (10, True),
        (2, 3): (10, True),
        (3, 4): (10, True),
                     
        (1, 5): (10, False),
        (2, 5): (10, False),
        (2, 6): (10, False),
        (3, 6): (10, False),
        (3, 7): (10, False),
        (4, 7): (10, False),

        (5, 6): (10, False),
        (6, 7): (10, False),
    }

    b_nodes = {
        1: pymunk.Vec2d(100, 200),
        2: pymunk.Vec2d(220, 200),
        3: pymunk.Vec2d(290, 200),
        4: pymunk.Vec2d(410, 200),
        5: pymunk.Vec2d(490, 200),
        6: pymunk.Vec2d(600, 200),
    }

    b_static_nodes = {1, 6}

    b_edges = {
        (1, 2): (10, True),
        (2, 3): (10, True),
        (3, 4): (10, True),
        (4, 5): (10, True),
        (5, 6): (10, True),
    }

    population = [
        Bridge(a_nodes, a_static_nodes, a_edges),
        Bridge(b_nodes, b_static_nodes, b_edges),
    ]

    tens = [] # tension, bridge from population

    for gen in range(1, 1 + 100):
        for bridge in population:
            #tens.append((comp_max_tension(bridge, (100, 180), 600, screen, draw_options, clock), bridge))
            tens.append((comp_max_tension(bridge, (100, 180), 600, screen, draw_options, None), bridge))

        tens.sort(key=lambda pair: pair[0])

        max_pop_size = 5
        if len(tens) > max_pop_size:
            tens = tens[:max_pop_size]

        population = []
        for i in range(len(tens)):
            for j in range(i + 1, len(tens)):
                a_bridge = tens[i][1]
                b_bridge = tens[j][1]
                population.append(Bridge(cross_nodes(a_bridge.nodes, b_bridge.nodes, a_bridge.static_nodes), a_bridge.static_nodes, a_bridge.edges))

        print(f'[{gen}]\tmin_tens: {tens[0][0]}')

    pygame.quit()

if __name__ == "__main__":
    main()
