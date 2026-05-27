import numpy as np
import math

def solve_truss(truss):
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

    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and beam.is_broken:
            continue  
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        L = truss.get_beam_length(beam)
        if L < 1e-3:
            continue
        dx_pixels = node_b.x - node_a.x
        dy_pixels = node_b.y - node_a.y
        pixel_length = math.hypot(dx_pixels, dy_pixels)
        cos_theta = dx_pixels / pixel_length
        sin_theta = -dy_pixels / pixel_length  
        k_element = (beam.area * beam.modulus) / L
        c2 = cos_theta ** 2
        s2 = sin_theta ** 2
        cs = cos_theta * sin_theta
        k_local = np.array([
            [ c2,  cs, -c2, -cs],
            [ cs,  s2, -cs, -s2],
            [-c2, -cs,  c2,  cs],
            [-cs, -s2,  cs,  s2]
        ]) * k_element
        dof = [idx_a * 2, idx_a * 2 + 1, idx_b * 2, idx_b * 2 + 1]
        for row_local, row_global in enumerate(dof):
            for col_local, col_global in enumerate(dof):
                K_global[row_global, col_global] += k_local[row_local, col_local]

    K_orig = K_global.copy()
    F_orig = F_global.copy()

    for i, node in enumerate(truss.nodes):
        if node.is_anchor_x or not node_has_connections[i]:
            dof_x = i * 2
            K_global[dof_x, :] = 0
            K_global[:, dof_x] = 0
            K_global[dof_x, dof_x] = 1.0
            F_global[dof_x] = 0
        if node.is_anchor_y or not node_has_connections[i]:
            dof_y = i * 2 + 1
            K_global[dof_y, :] = 0
            K_global[:, dof_y] = 0
            K_global[dof_y, dof_y] = 1.0
            F_global[dof_y] = 0

    try:
        displacements = np.linalg.solve(K_global, F_global)
        truss.displacements = displacements
        truss.is_stable = True
    except np.linalg.LinAlgError:
        truss.displacements = None
        truss.is_stable = False
        for beam in truss.beams:
            beam.force = 0.0
            beam.stress = 0.0
        return

    reactions = K_orig.dot(displacements) - F_orig
    for i, node in enumerate(truss.nodes):
        if node.is_anchor_x:
            node.rx = reactions[i * 2]
        if node.is_anchor_y:
            node.ry = -reactions[i * 2 + 1]

    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and beam.is_broken:
            beam.force = 0.0
            beam.stress = 0.0
            continue
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        L = truss.get_beam_length(beam)
        dx_pixels = node_b.x - node_a.x
        dy_pixels = node_b.y - node_a.y
        pixel_length = math.hypot(dx_pixels, dy_pixels)
        cos_theta = dx_pixels / pixel_length
        sin_theta = -dy_pixels / pixel_length
        u_ax = displacements[idx_a * 2]
        u_ay = displacements[idx_a * 2 + 1]
        u_bx = displacements[idx_b * 2]
        u_by = displacements[idx_b * 2 + 1]
        delta = (u_bx - u_ax) * cos_theta + (u_by - u_ay) * sin_theta
        beam.stress = (delta / L) * beam.modulus
        beam.force = beam.stress * beam.area

def calculate_benchmark_metrics(truss):
    if len(truss.nodes) != 3 or len(truss.beams) != 3:
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