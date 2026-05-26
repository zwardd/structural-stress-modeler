import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.is_anchor_x = False
        self.is_anchor_y = False
        self.load_x = 0.0
        self.load_y = 0.0
        self.rx = 0.0
        self.ry = 0.0

    def toggle_support(self):
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
    def __init__(self, node_a_index, node_b_index, material_name="Steel"):
        self.node_a = node_a_index
        self.node_b = node_b_index
        self.material = material_name
        self.area = 2.5e-3          
        self.inertia = 2.1e-6       
        self.update_material_properties(material_name)
        self.stress = 0.0           
        self.force = 0.0            
        self.is_broken = False

    def update_material_properties(self, material_name):
        self.material = material_name
        if material_name == "Aluminum":
            self.modulus = 70e9
        elif material_name == "Titanium":
            self.modulus = 114e9
        else:
            self.modulus = 200e9

    def adjust_area(self, delta):
        self.area = max(5.0e-4, min(1.0e-2, self.area + delta))
        self.inertia = 0.336 * (self.area ** 2)
        self.is_broken = False

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []
        self.active_material = "Steel"
        self.displacements = None
        self.is_stable = True

    def set_material(self, material_name):
        if material_name in ["Steel", "Aluminum", "Titanium"]:
            self.active_material = material_name

    def add_node(self, x, y, snap_enabled=False, grid_size=20):
        if snap_enabled:
            x = round((x - 160) / grid_size) * grid_size + 160
            y = round((y - 20) / grid_size) * grid_size + 20
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
        new_beam = Beam(index_a, index_b, self.active_material)
        self.beams.append(new_beam)
        return len(self.beams) - 1

    def get_beam_length(self, beam):
        node_a = self.nodes[beam.node_a]
        node_b = self.nodes[beam.node_b]
        return math.hypot(node_b.x - node_a.x, node_b.y - node_a.y)

    def clear(self):
        self.nodes = []
        self.beams = []
        self.displacements = None
        self.is_stable = True