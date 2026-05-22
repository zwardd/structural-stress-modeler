import pygame
import sys
from truss_model import TrussSystem
from matrix_solver import solve_truss

pygame.init()

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

# Color Palette
COLOR_BACKGROUND = (24, 24, 27)      
COLOR_SIM_ZONE   = (33, 33, 38)      
COLOR_PANEL_BG   = (18, 18, 20)      
COLOR_UI_BORDER  = (63, 63, 70)      
COLOR_TEXT_MAIN  = (244, 244, 245)    
COLOR_NODE       = (59, 130, 246)   
COLOR_BEAM       = (113, 113, 122)  
COLOR_PIN        = (34, 197, 94)    
COLOR_ROLLER     = (234, 179, 8)    
COLOR_LOAD       = (239, 68, 68)    

font_header = pygame.font.SysFont("Helvetica", 24, bold=True)
font_body = pygame.font.SysFont("Helvetica", 16)

clock = pygame.time.Clock()
truss = TrussSystem()

selected_node = None
CLICK_TOLERANCE = 12
NODE_RADIUS = 6

# Structural steel yield limit point (250 Megapascals)
STEEL_YIELD_STRESS = 250e6

is_running = True
while is_running:
    
    sim_rect = pygame.Rect(20, 20, 680, 660)
    panel_rect = pygame.Rect(720, 20, 260, 660)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            if not sim_rect.collidepoint(mouse_pos):
                continue

            clicked_node_idx = None
            for i, node in enumerate(truss.nodes):
                distance = pygame.math.Vector2(node.x, node.y).distance_to(mouse_pos)
                if distance < CLICK_TOLERANCE:
                    clicked_node_idx = i
                    break

            if event.button == 1:  
                keys = pygame.key.get_pressed()
                if keys[pygame.K_l] and clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].load_y += 10000.0
                    selected_node = None
                else:
                    if clicked_node_idx is not None:
                        if selected_node is None:
                            selected_node = clicked_node_idx
                        else:
                            truss.add_beam(selected_node, clicked_node_idx)
                            selected_node = None
                    else:
                        truss.add_node(mouse_pos[0], mouse_pos[1])
                        selected_node = None

            elif event.button == 3:  
                if clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].toggle_support()
                    selected_node = None

    # Compute Global Matrix Displacements and Element Tension/Compression Stresses
    solve_truss(truss)

    screen.fill(COLOR_BACKGROUND)

    # Render Simulation Canvas
    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    # Render Active Beams
    for beam in truss.beams:
        start_node = truss.nodes[beam.node_a]
        end_node = truss.nodes[beam.node_b]
        pygame.draw.line(screen, COLOR_BEAM, (start_node.x, start_node.y), (end_node.x, end_node.y), 3)

    # Render Active Nodes, Supports, and Loads
    total_applied_load = 0.0
    for i, node in enumerate(truss.nodes):
        total_applied_load += node.load_y
        
        if node.load_y > 0:
            arrow_start = (node.x, node.y - 10)
            arrow_end = (node.x, node.y - 35)
            pygame.draw.line(screen, COLOR_LOAD, arrow_start, arrow_end, 3)
            pygame.draw.polygon(screen, COLOR_LOAD, [arrow_start, (node.x - 5, node.y - 18), (node.x + 5, node.y - 18)])

        if node.is_anchor_x and node.is_anchor_y:
            pt1 = (node.x, node.y)
            pt2 = (node.x - 10, node.y + 14)
            pt3 = (node.x + 10, node.y + 14)
            pygame.draw.polygon(screen, COLOR_PIN, [pt1, pt2, pt3])
        elif node.is_anchor_y and not node.is_anchor_x:
            pt1 = (node.x, node.y)
            pt2 = (node.x - 10, node.y + 10)
            pt3 = (node.x + 10, node.y + 10)
            pygame.draw.polygon(screen, COLOR_ROLLER, [pt1, pt2, pt3])
            pygame.draw.line(screen, COLOR_ROLLER, (node.x - 10, node.y + 14), (node.x + 10, node.y + 14), 2)
        else:
            color = (234, 179, 8) if i == selected_node else COLOR_NODE
            pygame.draw.circle(screen, color, (node.x, node.y), NODE_RADIUS)

    # Render Side UI Panel
    pygame.draw.rect(screen, COLOR_PANEL_BG, panel_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, panel_rect, width=2, border_radius=8)

    # UI Text Display
    header_surface = font_header.render("SYSTEM METRICS", True, COLOR_TEXT_MAIN)
    screen.blit(header_surface, (740, 40))

    # Evaluate max stress utilization scaling factor across structural layout
    max_stress_observed = 0.0
    for beam in truss.beams:
        max_stress_observed = max(max_stress_observed, abs(beam.stress))
    
    utilization_pct = (max_stress_observed / STEEL_YIELD_STRESS) * 100.0
    safety_status = "NOMINAL"
    if utilization_pct > 100.0:
        safety_status = "CRITICAL FAILURE"
    elif utilization_pct > 75.0:
        safety_status = "WARNING"

    load_kn = total_applied_load / 1000.0
    metrics = [
        f"Total Nodes: {len(truss.nodes)}",
        f"Structural Beams: {len(truss.beams)}",
        f"Applied Load: {load_kn:.2f} kN",
        f"Max Material Stress: {utilization_pct:.1f}%",
        f"Safety Status: {safety_status}"
    ]

    text_y_position = 90
    for text_line in metrics:
        text_surface = font_body.render(text_line, True, COLOR_TEXT_MAIN)
        screen.blit(text_surface, (740, text_y_position))
        text_y_position += 30

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()