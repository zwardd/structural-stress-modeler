import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.is_anchor_x = False
        self.is_anchor_y = False
        self.load_x = 0.0
        self.load_y = 0.0

    def toggle_support(self):
        """Cycles through boundary constraints: Free -> Pin (Fixed) -> Roller -> Free"""
        if not self.is_anchor_x and not self.is_anchor_y:
            self.is_anchor_x = True
            self.is_anchor_y = True
        elif self.is_anchor_x and self.is_anchor_y:
            self.is_anchor_x = False
            self.is_anchor_y = True
        else:
            self.is_anchor_x = False
            self.is_anchor_y = False

class Beam:
    def __init__(self, node_a_index, node_b_index):
        self.node_a = node_a_index
        self.node_b = node_b_index
        
        self.area = 2.5e-3          # Cross-sectional area (meters squared)
        self.modulus = 200e9        # Young's Modulus (Pascals)
        self.stress = 0.0           # Internal stress running through beam (Pascals)
        self.force = 0.0            # Total internal axial load (Newtons)

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []

    def add_node(self, x, y):
        for node in self.nodes:
            if math.hypot(node.x - x, node.y - y) < 10:
                return None
        new_node = Node(x, y)
        self.nodes.append(new_node)
        return len(self.nodes) - 1

    def add_beam(self, index_a, index_b):
        if index_a == index_b:
            return None
        for beam in self.beams:
            if (beam.node_a == index_a and beam.node_b == index_b) or \
               (beam.node_a == index_b and beam.node_b == index_a):
                return None
        new_beam = Beam(index_a, index_b)
        self.beams.append(new_beam)
        return len(self.beams) - 1

    def get_beam_length(self, beam):
        node_a = self.nodes[beam.node_a]
        node_b = self.nodes[beam.node_b]
        return math.hypot(node_b.x - node_a.x, node_b.y - node_a.y)

    def clear(self):
        """Purges all structural entities from the tracking arrays to reset the canvas."""
        self.nodes = []
        self.beams = []