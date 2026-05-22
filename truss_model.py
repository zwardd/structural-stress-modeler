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
        if not self.is_anchor_x and not self.is_anchor_y:
            # Switch to Pin Support (Fixed X and Y)
            self.is_anchor_x = True
            self.is_anchor_y = True
        elif self.is_anchor_x and self.is_anchor_y:
            # Switch to Roller Support (Fixed Y only)
            self.is_anchor_x = False
            self.is_anchor_y = True
        else:
            # Reset to free node
            self.is_anchor_x = False
            self.is_anchor_y = False

class Beam:
    def __init__(self, node_a_idx, node_b_idx):
        self.node_a = node_a_idx
        self.node_b = node_b_idx
        self.area = 0.002        
        self.modulus = 200e9     
        self.force = 0.0         
        self.stress = 0.0        

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []

    def add_node(self, x, y):
        self.nodes.append(Node(x, y))

    def add_beam(self, a_idx, b_idx):
        if a_idx != b_idx:
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