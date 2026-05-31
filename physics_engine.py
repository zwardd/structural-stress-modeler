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

class PhysicsSimulation:
    def __init__(self, truss_system, gravity_mult=1.0, enable_gravity=True):
        self.truss = truss_system
        self.particles = []
        self.constraints = []
        self.sub_steps = 4
        self.constraint_cycle_base = 2
        self.constraint_cycle_max = 5
        self.constraint_error_threshold = 0.006
        self.dt = 1.0 / (60.0 * self.sub_steps)
        self.enable_gravity = enable_gravity
        self.gravity = 9.81 * gravity_mult if enable_gravity else 0.0
        self.max_length_error = 0.0
        self.avg_length_error = 0.0
        self.max_rel_length_error = 0.0
        self.avg_rel_length_error = 0.0
        self.used_constraint_cycles = 0
        self.init_simulation(truss_system)

    def init_simulation(self, truss):
        num_nodes = len(truss.nodes)
        node_masses = [0.1 for _ in range(num_nodes)]

        for beam in truss.beams:
            if beam.status == "FRACTURED":
                continue
            beam_mass = truss.get_beam_length(beam) * beam.area * beam.density
            node_masses[beam.node_a] += beam_mass / 2.0
            node_masses[beam.node_b] += beam_mass / 2.0

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

        for beam in truss.beams:
            if beam.status == "FRACTURED":
                continue
            dx = truss.nodes[beam.node_b].x - truss.nodes[beam.node_a].x
            dy = truss.nodes[beam.node_b].y - truss.nodes[beam.node_a].y
            rest_len = math.sqrt(dx * dx + dy * dy)
            c = PhysicsConstraint(beam.node_a, beam.node_b, rest_len, beam)
            self.constraints.append(c)

        self.particles.sort(key=lambda p: p.node_idx)
        self.constraints.sort(key=lambda c: (min(c.p1_idx, c.p2_idx), max(c.p1_idx, c.p2_idx)))

    def remove_constraints_for_beam(self, beam):
        self.constraints = [c for c in self.constraints if c.beam is not beam and c.beam != beam]

    def project_constraints(self):
        for c in self.constraints:
            p1 = self.particles[c.p1_idx]
            p2 = self.particles[c.p2_idx]
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 1e-6:
                continue
            diff = c.rest_length - dist
            percent = diff / dist * 0.5
            offset_x = dx * percent
            offset_y = dy * percent
            w1 = 0.0 if p1.is_anchor_x and p1.is_anchor_y else p1.inv_mass
            w2 = 0.0 if p2.is_anchor_x and p2.is_anchor_y else p2.inv_mass
            w_sum = w1 + w2
            if w_sum < 1e-8:
                continue
            if not p1.is_anchor_x:
                p1.x -= offset_x * (w1 / w_sum)
            if not p1.is_anchor_y:
                p1.y -= offset_y * (w1 / w_sum)
            if not p2.is_anchor_x:
                p2.x += offset_x * (w2 / w_sum)
            if not p2.is_anchor_y:
                p2.y += offset_y * (w2 / w_sum)

        for c in reversed(self.constraints):
            p1 = self.particles[c.p1_idx]
            p2 = self.particles[c.p2_idx]
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 1e-6:
                continue
            diff = c.rest_length - dist
            percent = diff / dist * 0.5
            offset_x = dx * percent
            offset_y = dy * percent
            w1 = 0.0 if p1.is_anchor_x and p1.is_anchor_y else p1.inv_mass
            w2 = 0.0 if p2.is_anchor_x and p2.is_anchor_y else p2.inv_mass
            w_sum = w1 + w2
            if w_sum < 1e-8:
                continue
            if not p1.is_anchor_x:
                p1.x -= offset_x * (w1 / w_sum)
            if not p1.is_anchor_y:
                p1.y -= offset_y * (w1 / w_sum)
            if not p2.is_anchor_x:
                p2.x += offset_x * (w2 / w_sum)
            if not p2.is_anchor_y:
                p2.y += offset_y * (w2 / w_sum)

    def compute_constraint_length_errors(self):
        total_abs = 0.0
        total_rel = 0.0
        count = 0
        max_abs = 0.0
        max_rel = 0.0
        for c in self.constraints:
            p1 = self.particles[c.p1_idx]
            p2 = self.particles[c.p2_idx]
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            curr_len = math.sqrt(dx * dx + dy * dy)
            abs_err = abs(curr_len - c.rest_length)
            rel_err = abs_err / c.rest_length if c.rest_length > 0 else 0.0
            total_abs += abs_err
            total_rel += rel_err
            max_abs = max(max_abs, abs_err)
            max_rel = max(max_rel, rel_err)
            count += 1
        self.max_length_error = max_abs
        self.avg_length_error = total_abs / count if count else 0.0
        self.max_rel_length_error = max_rel
        self.avg_rel_length_error = total_rel / count if count else 0.0
        return max_rel

    def step(self, gravity_mult):
        self.gravity = 9.81 * gravity_mult if self.enable_gravity else 0.0
        self.constraints = [c for c in self.constraints if c.beam.status != "FRACTURED"]
        self.constraints.sort(key=lambda c: (min(c.p1_idx, c.p2_idx), max(c.p1_idx, c.p2_idx)))

        for _ in range(self.sub_steps):
            for p in self.particles:
                node = self.truss.nodes[p.node_idx]
                if not p.is_anchor_x:
                    p.vx += (node.load_x * p.inv_mass) * self.dt
                if not p.is_anchor_y:
                    p.vy += (self.gravity + node.load_y * p.inv_mass) * self.dt
                p.px = p.x
                p.py = p.y
                if not p.is_anchor_x:
                    p.x += p.vx * self.dt
                if not p.is_anchor_y:
                    p.y += p.vy * self.dt

            self.used_constraint_cycles = 0
            for _ in range(self.constraint_cycle_base):
                self.project_constraints()
                self.used_constraint_cycles += 1

            current_error = self.compute_constraint_length_errors()
            while current_error > self.constraint_error_threshold and self.used_constraint_cycles < self.constraint_cycle_max:
                self.project_constraints()
                self.used_constraint_cycles += 1
                current_error = self.compute_constraint_length_errors()

            for p in self.particles:
                p.vx = (p.x - p.px) / self.dt
                p.vy = (p.y - p.py) / self.dt

    def sync_to_truss(self, truss):
        for p in self.particles:
            node = truss.nodes[p.node_idx]
            node.x = p.x
            node.y = p.y
        for c in self.constraints:
            dx = truss.nodes[c.p2_idx].x - truss.nodes[c.p1_idx].x
            dy = truss.nodes[c.p2_idx].y - truss.nodes[c.p1_idx].y
            curr_len = math.sqrt(dx * dx + dy * dy)
            if c.rest_length > 1e-6:
                strain = (curr_len - c.rest_length) / c.rest_length
                c.beam.stress = strain * c.beam.modulus
                c.beam.force = c.beam.stress * c.beam.area