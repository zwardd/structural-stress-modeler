import math

class Node:
    def __init__(self, x, y):
        self.x = x  # Raw screen pixels
        self.y = y  # Raw screen pixels
        self.is_anchor_x = False
        self.is_anchor_y = False
        self.load_x = 0.0  # Force in Newtons
        self.load_y = 0.0  # Force in Newtons
        self.rx = 0.0      # Reaction in Newtons
        self.ry = 0.0      # Reaction in Newtons

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
        self.profile = "Square Tube"
        self.dim_w = 0.05  # Outer width/diameter in meters
        self.dim_t = 0.005 # Wall thickness in meters
        self.area = 0.0    # Cross-sectional area in m²
        self.inertia = 0.0 # Moment of inertia in m⁴
        self.modulus = 200e9 # Elastic Modulus in Pa
        self.stress = 0.0   # Stress in Pa
        self.force = 0.0    # Axial force in Newtons
        self.is_broken = False
        self.broken_at_gravity = None
        
        self.update_material_properties(material_name)
        self.recalculate_geometry()

    def update_material_properties(self, material_name):
        self.material = material_name
        if material_name == "Aluminum":
            self.modulus = 70e9
        elif material_name == "Titanium":
            self.modulus = 114e9
        else:
            self.modulus = 200e9

    def adjust_dimension(self, delta):
        self.dim_w = max(0.01, min(0.3, self.dim_w + delta))
        if self.profile in ["Square Tube", "H-Beam"]:
            self.dim_t = max(0.002, min(self.dim_w * 0.4, self.dim_t + delta * 0.1))
        self.recalculate_geometry()

    def cycle_profile(self):
        profiles = ["Square Tube", "H-Beam", "Solid Bar"]
        idx = (profiles.index(self.profile) + 1) % len(profiles)
        self.profile = profiles[idx]
        self.recalculate_geometry()

    def recalculate_geometry(self):
        w = self.dim_w
        t = self.dim_t
        if self.profile == "Square Tube":
            self.area = (w * w) - ((w - 2 * t) * (w - 2 * t))
            self.inertia = ((w * w * w * w) - ((w - 2 * t) * (w - 2 * t) * (w - 2 * t) * (w - 2 * t))) / 12.0
        elif self.profile == "H-Beam":
            h = w
            b = w
            tw = t
            tf = t
            self.area = 2 * (b * tf) + (h - 2 * tf) * tw
            self.inertia = ((b * (h * h * h)) - ((b - tw) * ((h - 2 * tf) * (h - 2 * tf) * (h - 2 * tf)))) / 12.0
        else:
            r = w / 2.0
            self.area = math.pi * (r * r)
            self.inertia = (math.pi * (r * r * r * r)) / 4.0

class TrussSystem:
    # 20 pixels = 0.25 meters -> 1 pixel = 0.0125 meters -> 80 pixels = 1.0 meter
    PIXELS_PER_METER = 80.0

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
        """Returns the actual structural length of the element in METERS."""
        node_a = self.nodes[beam.node_a]
        node_b = self.nodes[beam.node_b]
        pixel_dist = math.hypot(node_b.x - node_a.x, node_b.y - node_a.y)
        return pixel_dist / self.PIXELS_PER_METER

    def clear(self):
        self.nodes.clear()
        self.beams.clear()
        self.displacements = None
        self.is_stable = True