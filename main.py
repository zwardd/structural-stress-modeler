import pygame
import sys
import math
from truss_model import TrussSystem
from matrix_solver import solve_truss

pygame.init()

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

COLOR_BACKGROUND = (24, 24, 27)      
COLOR_SIM_ZONE   = (33, 33, 38)      
COLOR_PANEL_BG   = (18, 18, 20)      
COLOR_UI_BORDER  = (63, 63, 70)      
COLOR_TEXT_MAIN  = (244, 244, 245)    
COLOR_TEXT_MUTED = (161, 161, 170)
COLOR_NODE       = (59, 130, 246)   
COLOR_PIN        = (34, 197, 94)    
COLOR_ROLLER     = (234, 179, 8)    
COLOR_LOAD       = (239, 68, 68)    
COLOR_HIGHLIGHT  = (255, 255, 255)  

COLOR_ZERO_LOAD = (144, 238, 144)   
COLOR_MID_LOAD  = (234, 179, 8)     
COLOR_MAX_LOAD  = (239, 68, 68)     

font_header = pygame.font.SysFont("Helvetica", 18, bold=True)
font_body = pygame.font.SysFont("Helvetica", 14)

clock = pygame.time.Clock()
truss = TrussSystem()

active_node_bnd = None  
selected_node_idx = None 
selected_beam_idx = None 

CLICK_TOLERANCE = 12
NODE_RADIUS = 6
STEEL_YIELD_STRESS = 250e6

def get_stress_color(stress_value, yield_limit):
    utilization = min(abs(stress_value) / yield_limit, 1.0)
    if utilization < 0.5:
        t = utilization / 0.5
        r = int(COLOR_ZERO_LOAD[0] + (COLOR_MID_LOAD[0] - COLOR_ZERO_LOAD[0]) * t)
        g = int(COLOR_ZERO_LOAD[1] + (COLOR_MID_LOAD[1] - COLOR_ZERO_LOAD[1]) * t)
        b = int(COLOR_ZERO_LOAD[2] + (COLOR_MID_LOAD[2] - COLOR_ZERO_LOAD[2]) * t)
    else:
        t = (utilization - 0.5) / 0.5
        r = int(COLOR_MID_LOAD[0] + (COLOR_MAX_LOAD[0] - COLOR_MID_LOAD[0]) * t)
        g = int(COLOR_MID_LOAD[1] + (COLOR_MAX_LOAD[1] - COLOR_MAX_LOAD[1]) * t)
        b = int(COLOR_MID_LOAD[2] + (COLOR_MAX_LOAD[2] - COLOR_MAX_LOAD[2]) * t)
    return (r, g, b)

def point_to_line_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.hypot(px - closest_x, py - closest_y)

is_running = True
while is_running:
    
    sim_rect = pygame.Rect(20, 20, 960, 660)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                truss.clear()
                active_node_bnd = None
                selected_node_idx = None
                selected_beam_idx = None
                
            elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                if selected_beam_idx is not None:
                    truss.beams.pop(selected_beam_idx)
                    selected_beam_idx = None
                elif selected_node_idx is not None:
                    idx = selected_node_idx
                    truss.beams = [b for b in truss.beams if b.node_a != idx and b.node_b != idx]
                    for b in truss.beams:
                        if b.node_a > idx:
                            b.node_a -= 1
                        if b.node_b > idx:
                            b.node_b -= 1
                    truss.nodes.pop(idx)
                    selected_node_idx = None
                    active_node_bnd = None

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
                    active_node_bnd = None
                else:
                    if clicked_node_idx is not None:
                        selected_node_idx = clicked_node_idx
                        selected_beam_idx = None  
                        if active_node_bnd is None:
                            active_node_bnd = clicked_node_idx
                        else:
                            truss.add_beam(active_node_bnd, clicked_node_idx)
                            active_node_bnd = None
                    else:
                        clicked_beam_idx = None
                        for i, beam in enumerate(truss.beams):
                            na = truss.nodes[beam.node_a]
                            nb = truss.nodes[beam.node_b]
                            dist = point_to_line_distance(mouse_pos[0], mouse_pos[1], na.x, na.y, nb.x, nb.y)
                            if dist < 8:  
                                clicked_beam_idx = i
                                break
                        
                        if clicked_beam_idx is not None:
                            selected_beam_idx = clicked_beam_idx
                            selected_node_idx = None
                            active_node_bnd = None
                        else:
                            truss.add_node(mouse_pos[0], mouse_pos[1])
                            active_node_bnd = None
                            selected_node_idx = None
                            selected_beam_idx = None

            elif event.button == 3:  
                if clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].toggle_support()
                    active_node_bnd = None
                    selected_node_idx = clicked_node_idx
                    selected_beam_idx = None

    solve_truss(truss)

    screen.fill(COLOR_BACKGROUND)

    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    for i, beam in enumerate(truss.beams):
        start_node = truss.nodes[beam.node_a]
        end_node = truss.nodes[beam.node_b]
        beam_color = get_stress_color(beam.stress, STEEL_YIELD_STRESS)
        
        if i == selected_beam_idx:
            pygame.draw.line(screen, COLOR_HIGHLIGHT, (start_node.x, start_node.y), (end_node.x, end_node.y), 8)
            
        pygame.draw.line(screen, beam_color, (start_node.x, start_node.y), (end_node.x, end_node.y), 4)

    for i, node in enumerate(truss.nodes):
        if node.load_y > 0:
            arrow_start = (node.x, node.y - 10)
            arrow_end = (node.x, node.y - 35)
            pygame.draw.line(screen, COLOR_LOAD, arrow_start, arrow_end, 3)
            pygame.draw.polygon(screen, COLOR_LOAD, [arrow_start, (node.x - 5, node.y - 18), (node.x + 5, node.y - 18)])

        if i == selected_node_idx or i == active_node_bnd:
            pygame.draw.circle(screen, COLOR_HIGHLIGHT, (node.x, node.y), NODE_RADIUS + 4, width=2)

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
            color = (234, 179, 8) if i == active_node_bnd else COLOR_NODE
            pygame.draw.circle(screen, color, (node.x, node.y), NODE_RADIUS)

    if (selected_node_idx is not None) or (selected_beam_idx is not None):
        lines = []
        header_text = ""
        
        if selected_beam_idx is not None:
            beam = truss.beams[selected_beam_idx]
            header_text = f"BEAM #{selected_beam_idx} METRICS"
            length_m = truss.get_beam_length(beam) / 20.0  
            stress_m_pa = beam.stress / 1e6               
            force_k_n = beam.force / 1000.0               
            
            nature = "NEUTRAL"
            if beam.stress > 1e-2:
                nature = "TENSION"
            elif beam.stress < -1e-2:
                nature = "COMPRESSION"
                
            utilization_pct = (abs(beam.stress) / STEEL_YIELD_STRESS) * 100.0
            
            lines = [
                f"Length: {length_m:.2f} m",
                f"Force: {abs(force_k_n):.1f} kN",
                f"Stress: {abs(stress_m_pa):.1f} MPa",
                f"Type: {nature}",
                f"Load Capacity: {utilization_pct:.1f}%"
            ]
            
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = f"NODE #{selected_node_idx} METRICS"
            
            support_str = "Free Joint"
            if node.is_anchor_x and node.is_anchor_y:
                support_str = "Pin Support"
            elif node.is_anchor_y and not node.is_anchor_x:
                support_str = "Roller Support"
                
            load_k_n = node.load_y / 1000.0
            
            lines = [
                f"Coords: ({int(node.x)}, {int(node.y)})",
                f"Fixture: {support_str}",
                f"Force Vector: {load_k_n:.1f} kN"
            ]

        hud_w = 240
        hud_h = 45 + (len(lines) * 24)
        hud_x = sim_rect.right - hud_w - 15
        hud_y = sim_rect.top + 15

        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surface, (18, 18, 20, 220), (0, 0, hud_w, hud_h), border_radius=6)
        pygame.draw.rect(hud_surface, (63, 63, 70, 255), (0, 0, hud_w, hud_h), width=1, border_radius=6)

        header_surface = font_header.render(header_text, True, COLOR_TEXT_MAIN)
        hud_surface.blit(header_surface, (15, 12))

        local_y = 40
        for line in lines:
            if ":" in line:
                parts = line.split(":", 1)
                lbl_surface = font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                val_surface = font_body.render(parts[1], True, COLOR_TEXT_MAIN)
                hud_surface.blit(lbl_surface, (15, local_y))
                hud_surface.blit(val_surface, (15 + lbl_surface.get_width() + 4, local_y))
            else:
                text_surface = font_body.render(line, True, COLOR_TEXT_MAIN)
                hud_surface.blit(text_surface, (15, local_y))
            local_y += 24

        screen.blit(hud_surface, (hud_x, hud_y))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()