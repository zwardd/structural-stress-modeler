import numpy as np
import math

def solve_truss(truss, gravity_multiplier=0.0):
    num_nodes = len(truss.nodes)
    for node in truss.nodes:
        node.rx = 0.0
        node.ry = 0.0
        
    if num_nodes < 2 or len(truss.beams) == 0:
        for beam in truss.beams:
            beam.force = 0.0
            beam.stress = 0.0
        truss.displacements = None
        truss.is_stable = True  
        return

    matrix_dim = num_nodes * 2
    K_global = np.zeros((matrix_dim, matrix_dim))
    F_global = np.zeros(matrix_dim)

    node_has_connections = [False] * num_nodes
    for beam in truss.beams:
        if beam.status == "FRACTURED":
            continue
        node_has_connections[beam.node_a] = True
        node_has_connections[beam.node_b] = True

    for i, node in enumerate(truss.nodes):
        F_global[i * 2] = node.load_x
        F_global[i * 2 + 1] = -node.load_y

    g = 9.81 * gravity_multiplier
    for beam in truss.beams:
        if beam.status == "FRACTURED":
            continue  
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        
        L = truss.get_beam_length(beam)
        if L == 0:
            continue
            
        if truss.self_weight_enabled and gravity_multiplier > 0.0:
            beam_mass = L * beam.area * beam.density
            half_weight_n = (beam_mass * g) / 2.0
            F_global[idx_a * 2 + 1] -= half_weight_n
            F_global[idx_b * 2 + 1] -= half_weight_n
            
        cos_t = (node_b.x - node_a.x) / (L * truss.PIXELS_PER_METER)
        sin_t = -(node_b.y - node_a.y) / (L * truss.PIXELS_PER_METER)
        
        k_element = (beam.area * beam.modulus / L) * np.array([
            [ cos_t**2,  cos_t*sin_t, -cos_t**2, -cos_t*sin_t],
            [ cos_t*sin_t, sin_t**2,  -cos_t*sin_t, -sin_t**2],
            [-cos_t**2, -cos_t*sin_t,  cos_t**2,  cos_t*sin_t],
            [-cos_t*sin_t, -sin_t**2,  cos_t*sin_t,  sin_t**2]
        ])
        
        indices = [idx_a * 2, idx_a * 2 + 1, idx_b * 2, idx_b * 2 + 1]
        for r_local, r_global in enumerate(indices):
            for c_local, c_global in enumerate(indices):
                K_global[r_global, c_global] += k_element[r_local, c_local]

    free_dofs = []
    boundary_dofs = []
    
    for i, node in enumerate(truss.nodes):
        if not node_has_connections[i] and not node.is_anchor_x and not node.is_anchor_y:
            continue
        if node.is_anchor_x:
            boundary_dofs.append(i * 2)
        else:
            free_dofs.append(i * 2)
            
        if node.is_anchor_y:
            boundary_dofs.append(i * 2 + 1)
        else:
            free_dofs.append(i * 2 + 1)

    if len(free_dofs) == 0:
        truss.displacements = np.zeros(matrix_dim)
        truss.is_stable = True
        return

    K_ff = K_global[np.ix_(free_dofs, free_dofs)]
    F_f = F_global[free_dofs]

    try:
        if np.linalg.matrix_rank(K_ff) < len(free_dofs):
            truss.is_stable = False
            truss.displacements = None
            for beam in truss.beams:
                beam.force = 0.0
                beam.stress = 0.0
            return
            
        u_free = np.linalg.solve(K_ff, F_f)
        truss.is_stable = True
    except np.linalg.LinAlgError:
        truss.is_stable = False
        truss.displacements = None
        for beam in truss.beams:
            beam.force = 0.0
            beam.stress = 0.0
        return

    u_full = np.zeros(matrix_dim)
    u_full[free_dofs] = u_free
    truss.displacements = u_full

    F_reactions = np.dot(K_global, u_full) - F_global
    for i, node in enumerate(truss.nodes):
        if node.is_anchor_x:
            node.rx = F_reactions[i * 2]
        if node.is_anchor_y:
            node.ry = -F_reactions[i * 2 + 1]

    for beam in truss.beams:
        if beam.status == "FRACTURED":
            beam.force = 0.0
            beam.stress = 0.0
            continue
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        
        L = truss.get_beam_length(beam)
        cos_t = (node_b.x - node_a.x) / (L * truss.PIXELS_PER_METER)
        sin_t = -(node_b.y - node_a.y) / (L * truss.PIXELS_PER_METER)
        
        u_elem = np.array([
            u_full[idx_a * 2],
            u_full[idx_a * 2 + 1],
            u_full[idx_b * 2],
            u_full[idx_b * 2 + 1]
        ])
        
        transformation = np.array([-cos_t, -sin_t, cos_t, sin_t])
        delta_L = np.dot(transformation, u_elem)
        
        beam.force = (beam.area * beam.modulus / L) * delta_L
        beam.stress = beam.force / beam.area

def calculate_benchmark_metrics(truss):
    if len(truss.nodes) < 3 or len(truss.beams) < 3:
        return None
        
    diag_beam = None
    horiz_beam = None
    
    for b in truss.beams:
        if (b.node_a == 0 and b.node_b == 2) or (b.node_a == 2 and b.node_b == 0):
            diag_beam = b
        elif (b.node_a == 1 and b.node_b == 2) or (b.node_a == 2 and b.node_b == 1):
            horiz_beam = b
            
    if diag_beam is None or horiz_beam is None or truss.displacements is None:
        return None

    p_load = 50000.0
    l_m = 3.0
    a_m2 = diag_beam.area
    e_pa = diag_beam.modulus

    f_diag_theory = p_load * math.sqrt(2)
    f_horiz_theory = -p_load

    dx_theory = (p_load * l_m) / (a_m2 * e_pa)
    dy_theory = -(p_load * l_m * (1.0 + 2.0 * math.sqrt(2))) / (a_m2 * e_pa)

    f_diag_num = diag_beam.force
    f_horiz_num = horiz_beam.force
    
    dx_num = -truss.displacements[4]  
    dy_num = truss.displacements[5]

    def get_pct_error(numerical, theoretical):
        if theoretical == 0:
            return 0.0 if numerical == 0 else float('inf')
        return abs((numerical - theoretical) / theoretical) * 100.0

    return {
        "diag_force_theory": f_diag_theory,
        "diag_force_num": f_diag_num,
        "diag_force_err": get_pct_error(f_diag_num, f_diag_theory),
        "horiz_force_theory": f_horiz_theory,
        "horiz_force_num": f_horiz_num,
        "horiz_force_err": get_pct_error(f_horiz_num, f_horiz_theory),
        "dx_theory": dx_theory,
        "dx_num": dx_num,
        "dx_err": get_pct_error(dx_num, dx_theory),
        "dy_theory": dy_theory,
        "dy_num": dy_num,
        "dy_err": get_pct_error(dy_num, dy_theory)
    }