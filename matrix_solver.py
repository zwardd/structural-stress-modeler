import numpy as np
import math

def solve_truss(truss):
    num_nodes = len(truss.nodes)
    if num_nodes == 0 or len(truss.beams) == 0:
        return

    # Initialize a 2D global tracking grid
    matrix_dim = num_nodes * 2
    K_global = np.zeros((matrix_dim, matrix_dim))
    F_global = np.zeros(matrix_dim)

    # Populate the global force vector array
    for i, node in enumerate(truss.nodes):
        F_global[i * 2] = node.load_x
        F_global[i * 2 + 1] = -node.load_y  # Invert coordinate plane to match gravity

    # Assemble the global structural grid from beam elements
    for beam in truss.beams:
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]

        L = truss.get_beam_length(beam)
        if L < 1e-3:
            continue

        # Calculate local unit components
        cos_theta = (node_b.x - node_a.x) / L
        sin_theta = -(node_b.y - node_a.y) / L  

        # Element axial stiffness equation: k_local = AE / L
        k_element = (beam.area * beam.modulus) / L

        # Local coordinate translation matrix mapping
        c2 = cos_theta ** 2
        s2 = sin_theta ** 2
        cs = cos_theta * sin_theta

        k_local = np.array([
            [ c2,  cs, -c2, -cs],
            [ cs,  s2, -cs, -s2],
            [-c2, -cs,  c2,  cs],
            [-cs, -s2,  cs,  s2]
        ]) * k_element

        # Map local elements into global index positions
        dof = [idx_a * 2, idx_a * 2 + 1, idx_b * 2, idx_b * 2 + 1]
        for row_local, row_global in enumerate(dof):
            for col_local, col_global in enumerate(dof):
                K_global[row_global, col_global] += k_local[row_local, col_local]

    # Enforce boundary condition anchors to prevent matrix calculation failure
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

    # Solve the system equation matrix [F = K * U] using linear algebra mechanics
    try:
        displacements = np.linalg.solve(K_global, F_global)
    except np.linalg.LinAlgError:
        # Prevent runtime crashes if structural systems are unstable or floating
        return

    # Back-calculate internal stresses running through each structural beam
    for beam in truss.beams:
        idx_a = beam.node_a
        idx_b = beam.node_b
        node_a = truss.nodes[idx_a]
        node_b = truss.nodes[idx_b]

        L = truss.get_beam_length(beam)
        cos_theta = (node_b.x - node_a.x) / L
        sin_theta = -(node_b.y - node_a.y) / L

        # Extract specific nodal displacement elements
        u_ax = displacements[idx_a * 2]
        u_ay = displacements[idx_a * 2 + 1]
        u_bx = displacements[idx_b * 2]
        u_by = displacements[idx_b * 2 + 1]

        # Calculate structural elongation delta
        delta = (u_bx - u_ax) * cos_theta + (u_by - u_ay) * sin_theta
        
        # Stress (Pascals) = Strain * Young's Modulus
        beam.stress = (delta / L) * beam.modulus
        # Force (Newtons) = Stress * Cross-Sectional Area
        beam.force = beam.stress * beam.area