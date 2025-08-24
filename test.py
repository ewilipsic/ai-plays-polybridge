import pygame
import pymunk

class Bridge:
    def __init__(self, space, nodes, edges, static_nodes):
        """
        space: pymunk.Space - the physics space
        nodes: dict - {node_id: (x, y)} coordinates of nodes
        edges: list - {(node_a, node_b): mass} edges by node ids with mass
        static_nodes: list or set - node ids that are static with respect to world
        """
        self.space = space
        space.damping = 0.3
        space.iterations = 30000

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
            self.space.add(joint)
            self.joints[a] = joint
            
def draw_bridge(screen, bridge):
    screen.fill((255, 255, 255))

    for segment in bridge.shapes:
        body = segment.body
        p1 = body.position + segment.a.rotated(body.angle)
        p2 = body.position + segment.b.rotated(body.angle)
        pygame.draw.line(screen, (0, 0, 0), p1, p2, 5)

    for id, pos in bridge.nodes.items():
        if id in bridge.static_nodes:
            color = (255, 0, 0)
        else:
            color = (0, 255, 0)
        pygame.draw.circle(screen, color, pos, 7.5)

def main():
    pygame.init()

    width, height = 600, 400
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Pymunk Bridge Graph with Pygame")

    clock = pygame.time.Clock()
    space = pymunk.Space()
    space.gravity = (0, 900)  # Gravity pulling down

    # Define nodes and edges
    nodes = {
        1: pymunk.Vec2d(100, 200),
        2: pymunk.Vec2d(200, 200),
        3: pymunk.Vec2d(300, 200),
        4: pymunk.Vec2d(400, 200),
        #5: pymunk.Vec2d(150, 300),
    }

    edges = {
        (1, 2): 10,
        (2, 3): 10,
        (3, 4): 10,
        #(1, 5): 10,
    }

    static_nodes = {1, 4}

    bridge = Bridge(space, nodes, edges, static_nodes)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        space.step(1 / 60.0)

        draw_bridge(screen, bridge)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
