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
        if hasattr(beam, 'is_broken') and beam.is_broken:
            continue
        node_has_connections[beam.node_a] = True
        node_has_connections[beam.node_b] = True

    for i, node in enumerate(truss.nodes):
        F_global[i * 2] = node.load_x
        F_global[i * 2 + 1] = -node.load_y

    if truss.self_weight_enabled and gravity_multiplier > 0.0:
        g = 9.81 * gravity_multiplier
        for beam in truss.beams:
            if hasattr(beam, 'is_broken') and beam.is_broken:
                continue
            length_m = truss.get_beam_length(beam)
            weight = length_m * beam.area * beam.density * g
            half_weight = weight / 2.0
            F_global[beam.node_a * 2 + 1] -= half_weight
            F_global[beam.node_b * 2 + 1] -= half_weight

    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and beam.is_broken:
            continue  
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        
        dx = node_b.x - node_a.x
        dy = -(node_b.y - node_a.y)
        L = math.hypot(dx, dy) / truss.PIXELS_PER_METER
        
        if L < 1e-6:
            continue
            
        c = dx / (L * truss.PIXELS_PER_METER)
        s = dy / (L * truss.PIXELS_PER_METER)
        
        AE_over_L = (beam.area * beam.modulus) / L
        
        k_element = AE_over_L * np.array([
            [ c*c,  c*s, -c*c, -c*s],
            [ c*s,  s*s, -c*s, -s*s],
            [-c*c, -c*s,  c*c,  c*s],
            [-c*s, -s*s,  c*s,  s*s]
        ])
        
        indices = [idx_a*2, idx_a*2+1, idx_b*2, idx_b*2+1]
        for i in range(4):
            for j in range(4):
                K_global[indices[i], indices[j]] += k_element[i, j]

    boundary_conditions = []
    for i, node in enumerate(truss.nodes):
        if node.is_anchor_x:
            boundary_conditions.append(i * 2)
        if node.is_anchor_y:
            boundary_conditions.append(i * 2 + 1)
            
        if not node_has_connections[i]:
            if i * 2 not in boundary_conditions:
                boundary_conditions.append(i * 2)
            if i * 2 + 1 not in boundary_conditions:
                boundary_conditions.append(i * 2 + 1)

    free_dofs = [dof for dof in range(matrix_dim) if dof not in boundary_conditions]

    if len(free_dofs) == 0:
        truss.displacements = np.zeros(matrix_dim)
        truss.is_stable = True
        return

    K_ff = K_global[np.ix_(free_dofs, free_dofs)]
    F_f = F_global[free_dofs]

    try:
        if np.abs(np.linalg.det(K_ff)) < 1e-5:
            truss.is_stable = False
            truss.displacements = None
            return
        
        U_f = np.linalg.solve(K_ff, F_f)
        truss.is_stable = True
    except np.linalg.LinAlgError:
        truss.is_stable = False
        truss.displacements = None
        return

    U_global = np.zeros(matrix_dim)
    U_global[free_dofs] = U_f
    truss.displacements = U_global

    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and beam.is_broken:
            beam.force = 0.0
            beam.stress = 0.0
            continue
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        
        dx = node_b.x - node_a.x
        dy = -(node_b.y - node_a.y)
        L = math.hypot(dx, dy) / truss.PIXELS_PER_METER
        
        c = dx / (L * truss.PIXELS_PER_METER)
        s = dy / (L * truss.PIXELS_PER_METER)
        
        u_elem = np.array([
            U_global[idx_a * 2],
            U_global[idx_a * 2 + 1],
            U_global[idx_b * 2],
            U_global[idx_b * 2 + 1]
        ])
        
        transformation = np.array([-c, -s, c, s])
        delta_L = np.dot(transformation, u_elem)
        
        beam.stress = (beam.modulus * delta_L) / L
        beam.force = beam.stress * beam.area

    F_reactions = np.dot(K_global, U_global) - F_global
    for i, node in enumerate(truss.nodes):
        if node.is_anchor_x:
            node.rx = F_reactions[i * 2]
        if node.is_anchor_y:
            node.ry = -F_reactions[i * 2 + 1]

def calculate_benchmark_metrics(truss):
    if len(truss.nodes) != 3 or len(truss.beams) != 3:
        return None
        
    diag_beam = None
    horiz_beam = None
    for b in truss.beams:
        if b.profile != "Square Tube" or not math.isclose(b.dim_w, 0.05) or not math.isclose(b.dim_t, 0.005) or b.material != "Steel":
            return None
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