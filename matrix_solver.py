import numpy as np
import math

def solve_truss(truss, gravity_multiplier=0.0):
    num_nodes = len(truss.nodes)
    for node in truss.nodes:
        node.rx = 0.0
        node.ry = 0.0
        
    if num_nodes < 2 or (len(truss.beams) + len(truss.cables) + len(truss.roads) == 0):
        for elem in truss.beams + truss.cables + truss.roads:
            elem.force = 0.0
            elem.stress = 0.0
        truss.displacements = None
        truss.is_stable = True  
        return

    matrix_dim = num_nodes * 2
    g = 9.81 * gravity_multiplier
    
    active_cables = [True] * len(truss.cables)
    max_iterations = 5
    
    for iteration in range(max_iterations):
        K_global = np.zeros((matrix_dim, matrix_dim))
        F_global = np.zeros(matrix_dim)

        node_has_connections = [False] * num_nodes
        for elem in truss.beams + truss.cables + truss.roads:
            if elem.status == "FRACTURED": continue
            node_has_connections[elem.node_a] = True
            node_has_connections[elem.node_b] = True

        for i, node in enumerate(truss.nodes):
            F_global[i * 2] = node.load_x
            F_global[i * 2 + 1] = -node.load_y

        for elem_idx, elem in enumerate(truss.beams + truss.cables + truss.roads):
            if elem.status == "FRACTURED": continue  
                
            idx_a = elem.node_a
            idx_b = elem.node_b
            node_a = truss.nodes[idx_a]
            node_b = truss.nodes[idx_b]
            
            dx = node_b.x - node_a.x
            dy = -(node_b.y - node_a.y)
            L = math.hypot(dx, dy) * 0.0125
            
            if L < 1e-5: continue
                
            cos_t = dx / (L / 0.0125)
            sin_t = dy / (L / 0.0125)
            
            if g > 0.0 and truss.self_weight_enabled:
                elem_mass = L * elem.area * elem.density
                half_weight = (elem_mass * g) / 2.0
                F_global[idx_a * 2 + 1] -= half_weight
                F_global[idx_b * 2 + 1] -= half_weight

            stiffness = (elem.modulus * elem.area) / L
            
            is_cable = getattr(elem, "is_cable", False)
            if is_cable:
                cable_idx = elem_idx - len(truss.beams)
                if not active_cables[cable_idx]:
                    stiffness *= 1e-9 

            k_local = stiffness * np.array([
                [ cos_t**2,  cos_t*sin_t, -cos_t**2, -cos_t*sin_t],
                [ cos_t*sin_t,  sin_t**2, -cos_t*sin_t, -sin_t**2],
                [-cos_t**2, -cos_t*sin_t,  cos_t**2,  cos_t*sin_t],
                [-cos_t*sin_t, -sin_t**2,  cos_t*sin_t,  sin_t**2]
            ])
            
            indices = [idx_a * 2, idx_a * 2 + 1, idx_b * 2, idx_b * 2 + 1]
            for r_idx, r in enumerate(indices):
                for c_idx, c in enumerate(indices):
                    K_global[r, c] += k_local[r_idx, c_idx]

        fixed_dofs = []
        for i, node in enumerate(truss.nodes):
            if not node_has_connections[i]:
                fixed_dofs.append(i * 2)
                fixed_dofs.append(i * 2 + 1)
                continue
            if node.is_anchor_x: fixed_dofs.append(i * 2)
            if node.is_anchor_y: fixed_dofs.append(i * 2 + 1)

        free_dofs = [d for d in range(matrix_dim) if d not in fixed_dofs]
        
        if len(free_dofs) == 0:
            for elem in truss.beams + truss.cables + truss.roads:
                elem.force = 0.0
                elem.stress = 0.0
            truss.displacements = np.zeros(matrix_dim)
            truss.is_stable = True
            return

        K_free = K_global[np.ix_(free_dofs, free_dofs)]
        F_free = F_global[free_dofs]

        try:
            rcond_val = 1.0 / np.linalg.cond(K_free)
            if rcond_val < 1e-12 or not np.isfinite(rcond_val):
                raise np.linalg.LinAlgError
            u_free = np.linalg.solve(K_free, F_free)
            truss.is_stable = True
        except (np.linalg.LinAlgError, ValueError):
            for elem in truss.beams + truss.cables + truss.roads:
                elem.force = 0.0
                elem.stress = 0.0
            truss.displacements = None
            truss.is_stable = False
            return

        u_global = np.zeros(matrix_dim)
        u_global[free_dofs] = u_free
        truss.displacements = u_global

        F_reactions = np.dot(K_global, u_global) - F_global
        for i, node in enumerate(truss.nodes):
            if node.is_anchor_x: node.rx = F_reactions[i * 2]
            if node.is_anchor_y: node.ry = -F_reactions[i * 2 + 1]

        cables_changed = False
        for elem_idx, elem in enumerate(truss.beams + truss.cables + truss.roads):
            if elem.status == "FRACTURED":
                elem.force = 0.0
                elem.stress = 0.0
                continue
                
            idx_a = elem.node_a
            idx_b = elem.node_b
            node_a = truss.nodes[idx_a]
            node_b = truss.nodes[idx_b]
            
            dx = node_b.x - node_a.x
            dy = -(node_b.y - node_a.y)
            L = math.hypot(dx, dy) * 0.0125
            
            if L < 1e-5: continue
                
            cos_t = dx / (L / 0.0125)
            sin_t = dy / (L / 0.0125)
            
            u_a_x = u_global[idx_a * 2]
            u_a_y = u_global[idx_a * 2 + 1]
            u_b_x = u_global[idx_b * 2]
            u_b_y = u_global[idx_b * 2 + 1]
            
            dl = (u_b_x - u_a_x) * cos_t + (u_b_y - u_a_y) * sin_t
            elem.stress = elem.modulus * (dl / L)
            elem.force = elem.stress * elem.area
            
            if getattr(elem, "is_cable", False):
                cable_idx = elem_idx - len(truss.beams)
                if active_cables[cable_idx] and elem.force < -1e-5:
                    active_cables[cable_idx] = False
                    cables_changed = True
                elif not active_cables[cable_idx] and dl > 1e-6:
                    active_cables[cable_idx] = True
                    cables_changed = True
                    
        if not cables_changed:
            break

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