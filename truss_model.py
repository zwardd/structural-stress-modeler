import math
import json
from materials import MaterialManager, MATERIAL_SPECS

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
        self.peak_speed = 0.0

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
    def __init__(self, node_a, node_b, material="Steel"):
        self.node_a = node_a
        self.node_b = node_b
        self.material = material
        self.profile = "Square Tube"
        self.dim_w = 0.05
        self.dim_t = 0.004
        self.force = 0.0
        self.stress = 0.0
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization = 0.0
        self.lowest_fos = float('inf')
        self.update_material_properties(material)

    def update_material_properties(self, material):
        self.material = material
        specs = MaterialManager.get_material_specs(material)
        self.modulus = specs["modulus"]
        self.density = specs["density"]
        self.recalculate_geometry()

    def recalculate_geometry(self):
        self.area, self.inertia = MaterialManager.calculate_area_inertia(
            self.profile, self.dim_w, self.dim_t
        )

    def cycle_profile(self):
        self.profile = MaterialManager.get_next_profile(self.profile)
        self.recalculate_geometry()

    def adjust_dimension(self, delta):
        self.dim_w, self.dim_t = MaterialManager.adjust_dimensions(
            self.profile, self.dim_w, self.dim_t, delta
        )
        self.recalculate_geometry()

    def reset_status(self):
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization = 0.0
        self.lowest_fos = float('inf')

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
        beam.dim_t = data.get("dim_t", 0.004)
        beam.recalculate_geometry()
        return beam

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []
        self.displacements = None
        self.is_stable = True
        self.active_material = "Steel"
        self.self_weight_enabled = True
        self.sim_stats = {"peak_mass": 0.0, "lowest_fos": float('inf'), "highest_utilization": 0.0}

    def clear(self):
        self.nodes.clear()
        self.beams.clear()
        self.displacements = None
        self.is_stable = True
        self.sim_stats = {"peak_mass": 0.0, "lowest_fos": float('inf'), "highest_utilization": 0.0}

    def reset_sim_stats(self):
        self.sim_stats = {"peak_mass": 0.0, "lowest_fos": float('inf'), "highest_utilization": 0.0}
        for node in self.nodes:
            node.peak_speed = 0.0
        for beam in self.beams:
            beam.peak_utilization = 0.0
            beam.lowest_fos = float('inf')

    def set_material(self, material):
        if material in ["Steel", "Aluminum", "Titanium"]:
            self.active_material = material
            for beam in self.beams:
                beam.update_material_properties(material)
                beam.reset_status()

    def add_node(self, x, y, snap_enabled=True, grid_size=20):
        if snap_enabled:
            x = round(x / grid_size) * grid_size
            y = round(y / grid_size) * grid_size
        for node in self.nodes:
            if math.isclose(node.x, x, abs_tol=1e-3) and math.isclose(node.y, y, abs_tol=1e-3):
                return
        self.nodes.append(Node(x, y))

    def add_beam(self, node_a, node_b):
        if node_a == node_b:
            return
        for beam in self.beams:
            if (beam.node_a == node_a and beam.node_b == node_b) or (beam.node_a == node_b and beam.node_b == node_a):
                return
        self.beams.append(Beam(node_a, node_b, self.active_material))

    def get_beam_length(self, beam):
        na = self.nodes[beam.node_a]
        nb = self.nodes[beam.node_b]
        return math.hypot(nb.x - na.x, nb.y - na.y) * 0.0125

    def save_to_file(self, filename):
        project_data = {
            "self_weight_enabled": self.self_weight_enabled,
            "active_material": self.active_material,
            "nodes": [n.to_dict() for n in self.nodes],
            "beams": [b.to_dict() for b in self.beams]
        }
        try:
            with open(filename, "w") as f:
                json.dump(project_data, f, indent=4)
            return True
        except IOError:
            return False

    def load_from_file(self, filename):
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
            beam.update_material_properties("Steel")
            beam.profile = "Square Tube"
            beam.dim_w = 0.05
            beam.dim_t = 0.004
            beam.recalculate_geometry()

    def optimize_step(self, calculate_utilization_fn, solve_truss_fn, gravity_multiplier, allow_profile_switching, dynamic_eval_fn=None):
        solve_truss_fn(self, gravity_multiplier)
        is_mechanism = not self.is_stable
        
        if len(self.beams) == 0:
            return False
            
        if is_mechanism:
            if dynamic_eval_fn is None:
                return False
            dynamic_eval_fn(self, gravity_multiplier)
            
        TARGET_UTIL = 0.50
        TOLERANCE = 0.02
        MIN_UTIL = TARGET_UTIL - TOLERANCE
        step_changed = False
        
        for beam in self.beams:
            if beam.status == "FRACTURED":
                continue
                
            util = beam.peak_utilization if is_mechanism else calculate_utilization_fn(beam)
            if util == 0.0:
                continue
            
            if allow_profile_switching:
                best_profile = beam.profile
                min_area = float('inf')
                saved_w = beam.dim_w
                saved_t = beam.dim_t
                saved_profile = beam.profile
                
                for p_type in ["Square Tube", "H-Beam", "Solid Bar"]:
                    beam.profile = p_type
                    beam.dim_w = saved_w
                    beam.dim_t = saved_t
                    beam.recalculate_geometry()
                    
                    p_util = beam.peak_utilization if is_mechanism else calculate_utilization_fn(beam)
                    if p_util == 0.0:
                        continue
                    
                    if p_util > TARGET_UTIL:
                        delta = max(0.0002, min(0.004, (p_util - TARGET_UTIL) * 0.02))
                        test_w = min(0.3, beam.dim_w + delta)
                    elif p_util < MIN_UTIL:
                        delta = max(0.0001, min(0.002, (MIN_UTIL - p_util) * 0.01))
                        test_w = max(0.01, beam.dim_w - delta)
                    else:
                        test_w = beam.dim_w
                        
                    beam.dim_w = test_w
                    if beam.profile in ["Square Tube", "H-Beam"]:
                        beam.dim_t = max(0.002, min(beam.dim_w * 0.4, beam.dim_w * 0.1))
                    beam.recalculate_geometry()
                    
                    if beam.area < min_area:
                        min_area = beam.area
                        best_profile = p_type
                        
                beam.profile = saved_profile
                beam.dim_w = saved_w
                beam.dim_t = saved_t
                beam.recalculate_geometry()
                
                if beam.profile != best_profile:
                    beam.profile = best_profile
                    step_changed = True

            old_w = beam.dim_w
            util = beam.peak_utilization if is_mechanism else calculate_utilization_fn(beam)
            if util > TARGET_UTIL:
                delta = max(0.001, min(0.015, (util - TARGET_UTIL) * 0.02))
                beam.dim_w = min(0.3, beam.dim_w + delta)
            elif util < MIN_UTIL:
                delta = max(0.0005, min(0.008, (MIN_UTIL - util) * 0.01))
                beam.dim_w = max(0.01, beam.dim_w - delta)
            if beam.profile in ["Square Tube", "H-Beam"]:
                beam.dim_t = max(0.002, min(beam.dim_w * 0.4, beam.dim_w * 0.1))
            beam.recalculate_geometry()
            if not math.isclose(beam.dim_w, old_w, abs_tol=1e-6):
                step_changed = True
                
        return step_changed