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

    for i, node in enumerate(truss.nodes):
        F_global[i * 2] = node.load_x
        F_global[i * 2 + 1] = -node.load_y

    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and beam.is_broken:
            continue  # Completely bypass compiling broken elements
            
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]
        L = truss.get_beam_length(beam)
        if L < 1e-3:
            continue
        cos_theta = (node_b.x - node_a.x) / L
        sin_theta = -(node_b.y - node_a.y) / L  
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
        if node.is_anchor_x:
            dof_x = i * 2
            K_global[dof_x, :] = 0
            K_global[:, dof_x] = 0
            K_global[dof_x, dof_x] = 1.0
            F_global[dof_x] = 0
        if node.is_anchor_y:
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
        cos_theta = (node_b.x - node_a.x) / L
        sin_theta = -(node_b.y - node_a.y) / L
        u_ax = displacements[idx_a * 2]
        u_ay = displacements[idx_a * 2 + 1]
        u_bx = displacements[idx_b * 2]
        u_by = displacements[idx_b * 2 + 1]
        delta = (u_bx - u_ax) * cos_theta + (u_by - u_ay) * sin_theta
        beam.stress = (delta / L) * beam.modulus
        beam.force = beam.stress * beam.area