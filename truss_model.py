import math
import json

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

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "is_anchor_x": self.is_anchor_x,
            "is_anchor_y": self.is_anchor_y,
            "load_x": self.load_x,
            "load_y": self.load_y
        }

    @classmethod
    def from_dict(cls, data):
        node = cls(data["x"], data["y"])
        node.is_anchor_x = data.get("is_anchor_x", False)
        node.is_anchor_y = data.get("is_anchor_y", False)
        node.load_x = data.get("load_x", 0.0)
        node.load_y = data.get("load_y", 0.0)
        return node


class Beam:
    def __init__(self, node_a_index, node_b_index, material_name="Steel"):
        self.node_a = node_a_index
        self.node_b = node_b_index
        self.material = material_name
        self.profile = "Square Tube"
        self.dim_w = 0.05
        self.dim_t = 0.005
        self.area = 0.0
        self.inertia = 0.0
        self.modulus = 200e9
        self.density = 7850.0
        self.stress = 0.0
        self.force = 0.0
        self.is_broken = False
        self.broken_at_gravity = None
        
        self.update_material_properties(material_name)
        self.recalculate_geometry()

    def update_material_properties(self, material_name):
        self.material = material_name
        if material_name == "Aluminum":
            self.modulus = 70e9
            self.density = 2700.0
        elif material_name == "Titanium":
            self.modulus = 114e9
            self.density = 4430.0
        else:
            self.modulus = 200e9
            self.density = 7850.0

    def adjust_dimension(self, delta):
        self.dim_w = max(0.01, min(0.3, self.dim_w + delta))
        if self.profile == "Square Tube" or self.profile == "H-Beam":
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
            I_xx = ((b * (h * h * h)) - ((b - tw) * ((h - 2 * tf) * (h - 2 * tf) * (h - 2 * tf)))) / 12.0
            I_yy = (2 * tf * (b * b * b) + (h - 2 * tf) * (tw * tw * tw)) / 12.0
            self.inertia = min(I_xx, I_yy)
        else:
            r = w / 2.0
            self.area = math.pi * (r * r)
            self.inertia = (math.pi * (r * r * r * r)) / 4.0

    def to_dict(self):
        return {
            "node_a": self.node_a,
            "node_b": self.node_b,
            "material": self.material,
            "profile": self.profile,
            "dim_w": self.dim_w,
            "dim_t": self.dim_t
        }

    @classmethod
    def from_dict(cls, data):
        beam = cls(data["node_a"], data["node_b"], data.get("material", "Steel"))
        beam.profile = data.get("profile", "Square Tube")
        beam.dim_w = data.get("dim_w", 0.05)
        beam.dim_t = data.get("dim_t", 0.005)
        beam.recalculate_geometry()
        return beam


class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []
        self.active_material = "Steel"
        self.displacements = None
        self.is_stable = True
        self.PIXELS_PER_METER = 80.0
        self.self_weight_enabled = True

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
        pixel_dist = math.hypot(node_b.x - node_a.x, node_b.y - node_a.y)
        return pixel_dist / self.PIXELS_PER_METER

    def save_to_file(self, filename="project.json"):
        project_data = {
            "version": "1.0",
            "self_weight_enabled": self.self_weight_enabled,
            "active_material": self.active_material,
            "nodes": [node.to_dict() for node in self.nodes],
            "beams": [beam.to_dict() for beam in self.beams]
        }
        with open(filename, "w") as f:
            json.dump(project_data, f, indent=4)

    def load_from_file(self, filename="project.json"):
        try:
            with open(filename, "r") as f:
                project_data = json.load(f)
            
            self.clear()
            self.self_weight_enabled = project_data.get("self_weight_enabled", True)
            self.active_material = project_data.get("active_material", "Steel")
            
            for node_data in project_data.get("nodes", []):
                self.nodes.append(Node.from_dict(node_data))
                
            for beam_data in project_data.get("beams", []):
                self.beams.append(Beam.from_dict(beam_data))
                
            return True
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return False

    def load_benchmark_case(self):
        self.clear()
        self.active_material = "Steel"
        self.self_weight_enabled = False
        
        self.add_node(450, 150)
        self.add_node(450, 390)
        self.add_node(690, 390)
        
        self.nodes[0].is_anchor_x = True
        self.nodes[0].is_anchor_y = True
        self.nodes[1].is_anchor_x = True
        self.nodes[1].is_anchor_y = True
        
        self.nodes[2].load_y = 50000.0
        
        self.add_beam(0, 1)
        self.add_beam(1, 2)
        self.add_beam(0, 2)
        
        for beam in self.beams:
            beam.dim_w = 0.05
            beam.dim_t = 0.005
            beam.profile = "Square Tube"
            beam.update_material_properties("Steel")
            beam.recalculate_geometry()

    def clear(self):
        self.nodes.clear()
        self.beams.clear()
        self.displacements = None
        self.is_stable = True