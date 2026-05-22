import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.is_anchor_x = False
        self.is_anchor_y = False
        self.load_x = 0.0
        self.load_y = 0.0

class Beam:
    def __init__(self, node_a_idx, node_b_idx):
        self.node_a = node_a_idx
        self.node_b = node_b_idx
        self.area = 0.002        # Cross-sectional area in square meters
        self.modulus = 200e9     # Young's Modulus for structural steel (Pascals)
        self.force = 0.0         # Internal calculated force (Newtons)
        self.stress = 0.0        # Calculated internal stress (Pascals)

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []

    def add_node(self, x, y):
        self.nodes.append(Node(x, y))

    def add_beam(self, a_idx, b_idx):
        if a_idx != b_idx:
            # Prevent duplicate beam structures
            for beam in self.beams:
                if {beam.node_a, beam.node_b} == {a_idx, b_idx}:
                    return False
            self.beams.append(Beam(a_idx, b_idx))
            return True
        return False

    def get_beam_length(self, beam):
        na = self.nodes[beam.node_a]
        nb = self.nodes[beam.node_b]
        return math.hypot(nb.x - na.x, nb.y - na.y)