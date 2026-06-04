from physics_engine import PhysicsSimulation

class SimulationController:
    def __init__(self):
        self.state = "EDIT"
        self.speed = 1.0
        self.time_accumulator = 0.0
        self.saved_truss_state = None
        self.physics_sim = None

    def capture_state(self, truss):
        return [{"x": n.x, "y": n.y} for n in truss.nodes]

    def restore_state(self, truss, state):
        for i, s in enumerate(state):
            truss.nodes[i].x = s["x"]
            truss.nodes[i].y = s["y"]

    def play(self, truss, gravity_multiplier):
        if self.state == "EDIT":
            self.saved_truss_state = self.capture_state(truss)
            truss.reset_sim_stats()
            self.physics_sim = PhysicsSimulation(truss, gravity_mult=gravity_multiplier, enable_gravity=truss.self_weight_enabled)
            self.state = "PLAY"
            self.time_accumulator = 0.0
        elif self.state == "PAUSE":
            self.state = "PLAY"

    def pause(self):
        if self.state == "PLAY":
            self.state = "PAUSE"

    def reset(self, truss):
        if self.state != "EDIT":
            self.state = "EDIT"
            self.physics_sim = None
            if self.saved_truss_state:
                self.restore_state(truss, self.saved_truss_state)
            for b in truss.beams:
                b.reset_status()
            truss.reset_sim_stats()
            self.time_accumulator = 0.0
            
    def set_speed(self, speed):
        self.speed = speed