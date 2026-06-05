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
        self.trail = []

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
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history = []
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
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history.clear()

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

class Cable:
    def __init__(self, node_a, node_b, material="Steel Wire Rope"):
        self.node_a = node_a
        self.node_b = node_b
        self.material = material
        self.profile = "Solid Bar"
        self.dim_w = 0.02
        self.dim_t = 0.000 
        self.force = 0.0
        self.stress = 0.0
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history = []
        self.is_cable = True
        self.update_material_properties(material)

    def update_material_properties(self, material=None):
        if material is not None:
            self.material = material
        specs = MaterialManager.get_material_specs(self.material)
        self.modulus = specs["modulus"]
        self.density = specs["density"]
        self.recalculate_geometry()

    def recalculate_geometry(self):
        self.area, self.inertia = MaterialManager.calculate_area_inertia(
            self.profile, self.dim_w, self.dim_t
        )

    def adjust_dimension(self, delta):
        self.dim_w = max(0.005, min(0.32, self.dim_w + delta))
        self.recalculate_geometry()

    def reset_status(self):
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history.clear()

    def to_dict(self):
        return {
            "node_a": self.node_a,
            "node_b": self.node_b,
            "dim_w": self.dim_w,
            "material": self.material
        }

    @classmethod
    def from_dict(cls, data):
        mat = data.get("material", "Steel Wire Rope")
        if mat == "Wire Rope": mat = "Steel Wire Rope"
        cable = cls(data["node_a"], data["node_b"], mat)
        cable.dim_w = data.get("dim_w", 0.02)
        cable.recalculate_geometry()
        return cable

class Road:
    def __init__(self, node_a, node_b):
        self.node_a = node_a
        self.node_b = node_b
        self.material = "Asphalt Concrete"
        self.profile = "Solid Rectangular"
        self.dim_w = 4.0
        self.dim_t = 0.25
        self.area = self.dim_w * self.dim_t
        self.inertia = (self.dim_w * (self.dim_t ** 3)) / 12.0
        self.modulus = 30e9
        self.density = 2400
        self.force = 0.0
        self.stress = 0.0
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history = []
        self.is_road = True

    def reset_status(self):
        self.status = "NORMAL"
        self.is_broken = False
        self.broken_at_gravity = None
        self.peak_utilization_seen = 0.0
        self.minimum_fos_seen = float('inf')
        self.history.clear()

    def to_dict(self):
        return {
            "node_a": self.node_a,
            "node_b": self.node_b
        }

    @classmethod
    def from_dict(cls, data):
        return cls(data["node_a"], data["node_b"])

class TrussSystem:
    def __init__(self):
        self.nodes = []
        self.beams = []
        self.cables = []
        self.roads = []
        self.displacements = None
        self.is_stable = True
        self.active_material = "Steel"
        self.active_cable_material = "Steel Wire Rope"
        self.self_weight_enabled = True
        self.sim_stats = {"peak_mass": 0.0, "sim_time": 0.0}
        self.minimum_fos_recorded = float('inf')
        self.peak_utilization_recorded = 0.0

    def clear(self):
        self.nodes.clear()
        self.beams.clear()
        self.cables.clear()
        self.roads.clear()
        self.displacements = None
        self.is_stable = True
        self.sim_stats = {"peak_mass": 0.0, "sim_time": 0.0}
        self.minimum_fos_recorded = float('inf')
        self.peak_utilization_recorded = 0.0

    def reset_sim_stats(self):
        self.sim_stats = {"peak_mass": 0.0, "sim_time": 0.0}
        self.minimum_fos_recorded = float('inf')
        self.peak_utilization_recorded = 0.0
        for node in self.nodes:
            node.peak_speed = 0.0
            node.trail.clear()
        for elem in self.beams + self.cables + self.roads:
            elem.peak_utilization_seen = 0.0
            elem.minimum_fos_seen = float('inf')
            elem.history.clear()

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
        if node_a == node_b: return
        for elem in self.beams + self.cables + self.roads:
            if (elem.node_a == node_a and elem.node_b == node_b) or (elem.node_a == node_b and elem.node_b == node_a): return
        self.beams.append(Beam(node_a, node_b, self.active_material))

    def add_cable(self, node_a, node_b):
        if node_a == node_b: return
        for elem in self.beams + self.cables + self.roads:
            if (elem.node_a == node_a and elem.node_b == node_b) or (elem.node_a == node_b and elem.node_b == node_a): return
        self.cables.append(Cable(node_a, node_b, self.active_cable_material))

    def add_road(self, node_a, node_b):
        if node_a == node_b: return
        for elem in self.beams + self.cables + self.roads:
            if (elem.node_a == node_a and elem.node_b == node_b) or (elem.node_a == node_b and elem.node_b == node_a): return
        self.roads.append(Road(node_a, node_b))

    def get_beam_length(self, elem):
        na = self.nodes[elem.node_a]
        nb = self.nodes[elem.node_b]
        return math.hypot(nb.x - na.x, nb.y - na.y) * 0.0125

    def save_to_file(self, filename):
        project_data = {
            "self_weight_enabled": self.self_weight_enabled,
            "active_material": self.active_material,
            "active_cable_material": self.active_cable_material,
            "nodes": [n.to_dict() for n in self.nodes],
            "beams": [b.to_dict() for b in self.beams],
            "cables": [c.to_dict() for c in self.cables],
            "roads": [r.to_dict() for r in self.roads]
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
            self.active_cable_material = project_data.get("active_cable_material", "Steel Wire Rope")
            for node_data in project_data.get("nodes", []):
                self.nodes.append(Node.from_dict(node_data))
            for beam_data in project_data.get("beams", []):
                self.beams.append(Beam.from_dict(beam_data))
            for cable_data in project_data.get("cables", []):
                self.cables.append(Cable.from_dict(cable_data))
            for road_data in project_data.get("roads", []):
                self.roads.append(Road.from_dict(road_data))
            return True
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return False

    def load_benchmark_case(self):
        self.clear()
        self.active_material = "Steel"
        self.active_cable_material = "Steel Wire Rope"
        self.self_weight_enabled = False
        self.add_node(-120, -120)
        self.add_node(-120, 120)
        self.add_node(120, 120)
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
        
        if len(self.beams) + len(self.cables) + len(self.roads) == 0:
            return False
            
        if is_mechanism:
            if dynamic_eval_fn is None:
                return False
            dynamic_eval_fn(self, gravity_multiplier)
            
        TARGET_UTIL = 0.50
        TOLERANCE = 0.02
        MIN_UTIL = TARGET_UTIL - TOLERANCE
        step_changed = False
        
        for elem in self.beams + self.cables:
            if elem.status == "FRACTURED":
                continue
                
            util = elem.peak_utilization_seen if is_mechanism else calculate_utilization_fn(elem)
            if util == 0.0:
                continue
            
            is_cable = getattr(elem, "is_cable", False)
            
            if allow_profile_switching and not is_cable:
                best_profile = elem.profile
                min_area = float('inf')
                saved_w = elem.dim_w
                saved_t = elem.dim_t
                saved_profile = elem.profile
                
                for p_type in ["Square Tube", "H-Beam", "Solid Bar"]:
                    elem.profile = p_type
                    elem.dim_w = saved_w
                    elem.dim_t = saved_t
                    elem.recalculate_geometry()
                    
                    p_util = elem.peak_utilization_seen if is_mechanism else calculate_utilization_fn(elem)
                    if p_util == 0.0: continue
                    
                    if p_util > TARGET_UTIL:
                        delta = max(0.0002, min(0.004, (p_util - TARGET_UTIL) * 0.01))
                        test_w = min(0.3, elem.dim_w + delta)
                    elif p_util < MIN_UTIL:
                        delta = max(0.0001, min(0.002, (MIN_UTIL - p_util) * 0.005))
                        test_w = max(0.01, elem.dim_w - delta)
                    else:
                        test_w = elem.dim_w
                        
                    elem.dim_w = test_w
                    if elem.profile in ["Square Tube", "H-Beam"]:
                        elem.dim_t = max(0.002, min(elem.dim_w * 0.4, elem.dim_w * 0.1))
                    elem.recalculate_geometry()
                    
                    if elem.area < min_area:
                        min_area = elem.area
                        best_profile = p_type
                        
                elem.profile = saved_profile
                elem.dim_w = saved_w
                elem.dim_t = saved_t
                elem.recalculate_geometry()
                
                if elem.profile != best_profile:
                    elem.profile = best_profile
                    step_changed = True

            old_w = elem.dim_w
            util = elem.peak_utilization_seen if is_mechanism else calculate_utilization_fn(elem)
            if util > TARGET_UTIL:
                delta = max(0.0002, min(0.004, (util - TARGET_UTIL) * 0.01))
                elem.dim_w = min(0.3, elem.dim_w + delta)
            elif util < MIN_UTIL:
                delta = max(0.0001, min(0.002, (MIN_UTIL - util) * 0.005))
                elem.dim_w = max(0.01, elem.dim_w - delta)
            
            if not is_cable and elem.profile in ["Square Tube", "H-Beam"]:
                elem.dim_t = max(0.002, min(elem.dim_w * 0.4, elem.dim_w * 0.1))
            
            elem.recalculate_geometry()
            if not math.isclose(elem.dim_w, old_w, abs_tol=1e-6):
                step_changed = True
                
        return step_changed