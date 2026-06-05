import math
import numpy as np

class PhysicsParticle:
    def __init__(self, node_idx, x, y, inv_mass, is_anchor_x, is_anchor_y):
        self.node_idx = node_idx
        self.x = float(x)
        self.y = float(y)
        self.px = float(x)
        self.py = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.inv_mass = inv_mass
        self.is_anchor_x = is_anchor_x
        self.is_anchor_y = is_anchor_y

class PhysicsConstraint:
    def __init__(self, p1_idx, p2_idx, rest_length, beam):
        self.p1_idx = p1_idx
        self.p2_idx = p2_idx
        self.rest_length = float(rest_length)
        self.beam = beam
        self.lam = 0.0
        self.frame_force_acc = 0.0
        self.final_force = 0.0

class PhysicsSimulation:
    def __init__(self, truss_system, gravity_mult=1.0, enable_gravity=True):
        self.truss = truss_system
        self.particles = []
        self.constraints = []
        self.sub_steps = 12
        self.iterations = 8
        self.dt = 1.0 / (60.0 * self.sub_steps)
        self.enable_gravity = enable_gravity
        self.gravity = 9.81 * gravity_mult if enable_gravity else 0.0
        self.init_simulation(truss_system)

    def init_simulation(self, truss):
        num_nodes = len(truss.nodes)
        node_masses = [0.1 for _ in range(num_nodes)]

        for elem in truss.beams + truss.cables + truss.roads:
            if elem.status == "FRACTURED":
                continue
            elem_mass = truss.get_beam_length(elem) * elem.area * elem.density
            node_masses[elem.node_a] += elem_mass / 2.0
            node_masses[elem.node_b] += elem_mass / 2.0

        for i, node in enumerate(truss.nodes):
            m = node_masses[i]
            inv_m = 1.0 / m if m > 0.0 else 1.0
            p = PhysicsParticle(
                node_idx=i,
                x=node.x,
                y=node.y,
                inv_mass=inv_m,
                is_anchor_x=node.is_anchor_x,
                is_anchor_y=node.is_anchor_y,
            )
            self.particles.append(p)

        for elem in truss.beams + truss.cables + truss.roads:
            if elem.status == "FRACTURED":
                continue
            dx = truss.nodes[elem.node_b].x - truss.nodes[elem.node_a].x
            dy = truss.nodes[elem.node_b].y - truss.nodes[elem.node_a].y
            rest_len = math.sqrt(dx * dx + dy * dy)
            c = PhysicsConstraint(elem.node_a, elem.node_b, rest_len, elem)
            self.constraints.append(c)

        self.particles.sort(key=lambda p: p.node_idx)
        self.constraints.sort(key=lambda c: (min(c.p1_idx, c.p2_idx), max(c.p1_idx, c.p2_idx)))

    def remove_constraints_for_beam(self, elem):
        self.constraints = [c for c in self.constraints if c.beam is not elem and c.beam != elem]

    def step(self, gravity_mult):
        self.gravity = 9.81 * gravity_mult if self.enable_gravity else 0.0
        self.constraints = [c for c in self.constraints if c.beam.status != "FRACTURED"]
        self.constraints.sort(key=lambda c: (min(c.p1_idx, c.p2_idx), max(c.p1_idx, c.p2_idx)))

        dt_sq = self.dt * self.dt
        gravity_px = self.gravity * 80.0

        for c in self.constraints:
            c.frame_force_acc = 0.0

        for _ in range(self.sub_steps):
            for c in self.constraints:
                c.lam = 0.0

            for p in self.particles:
                node = self.truss.nodes[p.node_idx]
                
                ax_px = (node.load_x * p.inv_mass) * 80.0
                ay_px = (node.load_y * p.inv_mass) * 80.0
                
                if not p.is_anchor_x:
                    p.vx += ax_px * self.dt
                if not p.is_anchor_y:
                    p.vy += (gravity_px + ay_px) * self.dt
                    
                p.px = p.x
                p.py = p.y
                
                if not p.is_anchor_x:
                    p.x += p.vx * self.dt
                if not p.is_anchor_y:
                    p.y += p.vy * self.dt

            for _ in range(self.iterations):
                for c in self.constraints:
                    p1 = self.particles[c.p1_idx]
                    p2 = self.particles[c.p2_idx]
                    
                    dx_px = p2.x - p1.x
                    dy_px = p2.y - p1.y
                    dist_px = math.hypot(dx_px, dy_px)
                    
                    if dist_px < 1e-6:
                        continue
                        
                    dist_m = dist_px * 0.0125
                    rest_m = c.rest_length * 0.0125
                    C_m = dist_m - rest_m
                    
                    w1 = 0.0 if p1.is_anchor_x and p1.is_anchor_y else p1.inv_mass
                    w2 = 0.0 if p2.is_anchor_x and p2.is_anchor_y else p2.inv_mass
                    w_sum = w1 + w2
                    
                    if w_sum < 1e-10:
                        continue
                    
                    k = (c.beam.modulus * c.beam.area) / rest_m if rest_m > 1e-6 else 1e9
                    alpha = 1.0 / k
                    alpha_tilde = alpha / dt_sq
                    
                    delta_lam = (-C_m - alpha_tilde * c.lam) / (w_sum + alpha_tilde)
                    
                    if getattr(c.beam, "is_cable", False):
                        new_lam = min(0.0, c.lam + delta_lam)
                        delta_lam = new_lam - c.lam
                        c.lam = new_lam
                    else:
                        c.lam += delta_lam
                    
                    corr_px_1 = (-w1 * delta_lam) * 80.0
                    corr_px_2 = (w2 * delta_lam) * 80.0
                    
                    nx = dx_px / dist_px
                    ny = dy_px / dist_px
                    
                    if not p1.is_anchor_x: p1.x += corr_px_1 * nx
                    if not p1.is_anchor_y: p1.y += corr_px_1 * ny
                    if not p2.is_anchor_x: p2.x += corr_px_2 * nx
                    if not p2.is_anchor_y: p2.y += corr_px_2 * ny

            for p in self.particles:
                p.vx = ((p.x - p.px) / self.dt) * 0.9995
                p.vy = ((p.y - p.py) / self.dt) * 0.9995

            for c in self.constraints:
                c.frame_force_acc += (-c.lam / dt_sq)

        for c in self.constraints:
            c.final_force = c.frame_force_acc / self.sub_steps

    def sync_to_truss(self, truss):
        for p in self.particles:
            node = truss.nodes[p.node_idx]
            node.x = p.x
            node.y = p.y
            speed_m_s = math.hypot(p.vx, p.vy) * 0.0125
            node.peak_speed = max(node.peak_speed, speed_m_s)
            
        for c in self.constraints:
            c.beam.force = c.final_force
            c.beam.stress = c.beam.force / c.beam.area