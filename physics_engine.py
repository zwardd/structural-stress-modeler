import math
import numpy as np
from constants import WORKSPACE_LIMIT

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
    def __init__(self, p1_idx, p2_idx, rest_length):
        self.p1_idx = p1_idx
        self.p2_idx = p2_idx
        self.rest_length = float(rest_length)

class PhysicsSimulation:
    def __init__(self, truss_system, gravity_mult=1.0):
        self.particles = []
        self.constraints = []
        self.sub_steps = 8
        self.dt = 1.0 / (60.0 * self.sub_steps)
        self.gravity = 9.81 * gravity_mult
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
                is_anchor_y=node.is_anchor_y
            )
            self.particles.append(p)

        for beam in truss.beams:
            if beam.status == "FRACTURED":
                continue
            dx = truss.nodes[beam.node_b].x - truss.nodes[beam.node_a].x
            dy = truss.nodes[beam.node_b].y - truss.nodes[beam.node_a].y
            rest_len = math.sqrt(dx*dx + dy*dy)
            
            c = PhysicsConstraint(beam.node_a, beam.node_b, rest_len)
            self.constraints.append(c)

    def step(self):
        for _ in range(self.sub_steps):
            for p in self.particles:
                if not p.is_anchor_x:
                    p.vy += self.gravity * self.dt
                
                p.px = p.x
                p.py = p.y
                
                if not p.is_anchor_x:
                    p.x += p.vx * self.dt
                if not p.is_anchor_y:
                    p.y += p.vy * self.dt

            for c in self.constraints:
                p1 = self.particles[c.p1_idx]
                p2 = self.particles[c.p2_idx]
                
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                dist = math.sqrt(dx*dx + dy*dy)
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

            for p in self.particles:
                p.vx = (p.x - p.px) / self.dt
                p.vy = (p.y - p.py) / self.dt

    def sync_to_truss(self, truss):
        for p in self.particles:
            node = truss.nodes[p.node_idx]
            node.x = p.x
            node.y = p.y