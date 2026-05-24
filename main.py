import pygame
import sys
import math
from truss_model import TrussSystem
from matrix_solver import solve_truss

pygame.init()

WINDOW_WIDTH = 1140
WINDOW_HEIGHT = 700
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

COLOR_BACKGROUND = (24, 24, 27)      
COLOR_SIM_ZONE   = (33, 33, 38)      
COLOR_PANEL_BG   = (18, 18, 20)      
COLOR_UI_BORDER  = (63, 63, 70)      
COLOR_TEXT_MAIN  = (244, 244, 245)    
COLOR_TEXT_MUTED = (161, 161, 170)
COLOR_GRID       = (40, 40, 45)
COLOR_NODE       = (59, 130, 246)   
COLOR_PIN        = (34, 197, 94)    
COLOR_ROLLER     = (45, 212, 191)    
COLOR_LOAD       = (239, 68, 68)    
COLOR_HIGHLIGHT  = (255, 255, 255)  

COLOR_ZERO_LOAD = (144, 238, 144)   
COLOR_MID_LOAD  = (234, 179, 8)     
COLOR_MAX_LOAD  = (239, 68, 68)     

font_header = pygame.font.SysFont("Helvetica", 16, bold=True)
font_body = pygame.font.SysFont("Helvetica", 13)

clock = pygame.time.Clock()
truss = TrussSystem()

active_node_bnd = None  
selected_node_idx = None 
selected_beam_idx = None 

current_mode = "SELECT" 
grid_enabled = True
GRID_SIZE = 20
CLICK_TOLERANCE = 12
NODE_RADIUS = 6

MATERIAL_SPECS = {
    "Steel": {"yield": 250e6, "label": "Structural Steel"},
    "Aluminum": {"yield": 275e6, "label": "6061-T6 Aluminum"},
    "Titanium": {"yield": 880e6, "label": "Ti-6Al-4V Titanium"}
}

def get_stress_color(beam):
    if beam.force == 0.0:
        return COLOR_ZERO_LOAD
        
    specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
    utilization = abs(beam.stress) / specs["yield"]
    
    if utilization >= 1.0:
        return COLOR_MAX_LOAD

    if utilization < 0.5:
        t = utilization / 0.5
        r = int(COLOR_ZERO_LOAD[0] + (COLOR_MID_LOAD[0] - COLOR_ZERO_LOAD[0]) * t)
        g = int(COLOR_ZERO_LOAD[1] + (COLOR_MID_LOAD[1] - COLOR_ZERO_LOAD[1]) * t)
        b = int(COLOR_ZERO_LOAD[2] + (COLOR_MID_LOAD[2] - COLOR_ZERO_LOAD[2]) * t)
    else:
        t = (utilization - 0.5) / 0.5
        r = int(COLOR_MID_LOAD[0] + (COLOR_MAX_LOAD[0] - COLOR_MID_LOAD[0]) * t)
        g = int(COLOR_MID_LOAD[1] + (COLOR_MAX_LOAD[1] - COLOR_MID_LOAD[1]) * t)
        b = int(COLOR_MID_LOAD[2] + (COLOR_MAX_LOAD[2] - COLOR_MAX_LOAD[2]) * t)
        
    return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))

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

def draw_force_vector(surface, cx, cy, fx, fy):
    mag = math.hypot(fx, fy)
    if mag < 1e-1:
        return
    
    dx = fx / mag
    dy = fy / mag
    
    arrow_len = max(20, min(65, int(mag / 1000.0) * 1.5 + 20))
    
    end_x = cx
    end_y = cy
    start_x = cx - dx * arrow_len
    start_y = cy - dy * arrow_len
    
    pygame.draw.line(surface, COLOR_LOAD, (start_x, start_y), (end_x, end_y), 3)
    
    wing_len = 8
    angle = math.atan2(end_y - start_y, end_x - start_x)
    
    left_wing = (end_x - wing_len * math.cos(angle + 0.4), end_y - wing_len * math.sin(angle + 0.4))
    right_wing = (end_x - wing_len * math.cos(angle - 0.4), end_y - wing_len * math.sin(angle - 0.4))
    
    pygame.draw.polygon(surface, COLOR_LOAD, [(end_x, end_y), left_wing, right_wing])

is_running = True
while is_running:
    
    sidebar_rect = pygame.Rect(0, 0, 140, 700)
    sim_rect = pygame.Rect(160, 20, 960, 660)

    btn_select = pygame.Rect(15, 80, 110, 40)
    btn_node   = pygame.Rect(15, 135, 110, 40)
    btn_beam   = pygame.Rect(15, 190, 110, 40)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                truss.clear()
                active_node_bnd = None
                selected_node_idx = None
                selected_beam_idx = None
            elif event.key == pygame.K_g:
                grid_enabled = not grid_enabled
            elif event.key == pygame.K_1:
                truss.set_material("Steel")
            elif event.key == pygame.K_2:
                truss.set_material("Aluminum")
            elif event.key == pygame.K_3:
                truss.set_material("Titanium")
                
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
            
            if sidebar_rect.collidepoint(mouse_pos):
                if btn_select.collidepoint(mouse_pos):
                    current_mode = "SELECT"
                    active_node_bnd = None
                elif btn_node.collidepoint(mouse_pos):
                    current_mode = "NODE"
                    active_node_bnd = None
                    selected_node_idx = None
                    selected_beam_idx = None
                elif btn_beam.collidepoint(mouse_pos):
                    current_mode = "BEAM"
                    selected_node_idx = None
                    selected_beam_idx = None
                continue

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
                    if current_mode == "SELECT":
                        if clicked_node_idx is not None:
                            selected_node_idx = clicked_node_idx
                            selected_beam_idx = None
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
                            else:
                                selected_node_idx = None
                                selected_beam_idx = None
                                
                    elif current_mode == "NODE":
                        if clicked_node_idx is None:
                            truss.add_node(mouse_pos[0], mouse_pos[1], snap_enabled=grid_enabled, grid_size=GRID_SIZE)
                        else:
                            selected_node_idx = clicked_node_idx
                            
                    elif current_mode == "BEAM":
                        if clicked_node_idx is not None:
                            if active_node_bnd is None:
                                active_node_bnd = clicked_node_idx
                            else:
                                truss.add_beam(active_node_bnd, clicked_node_idx)
                                active_node_bnd = None
                        else:
                            active_node_bnd = None

            elif event.button == 3:  
                if clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].toggle_support()
                    active_node_bnd = None
                    selected_node_idx = clicked_node_idx
                    selected_beam_idx = None

    if selected_node_idx is not None and current_mode == "SELECT":
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            truss.nodes[selected_node_idx].load_y -= 1000.0
        if keys[pygame.K_DOWN]:
            truss.nodes[selected_node_idx].load_y += 1000.0
        if keys[pygame.K_LEFT]:
            truss.nodes[selected_node_idx].load_x -= 1000.0
        if keys[pygame.K_RIGHT]:
            truss.nodes[selected_node_idx].load_x += 1000.0

    solve_truss(truss)

    screen.fill(COLOR_BACKGROUND)
    
    pygame.draw.rect(screen, COLOR_PANEL_BG, sidebar_rect)
    pygame.draw.line(screen, COLOR_UI_BORDER, (140, 0), (140, 700), 2)
    
    lbl_tools = font_header.render("TOOLBOX", True, COLOR_TEXT_MAIN)
    screen.blit(lbl_tools, (15, 30))

    for btn, label, mode_str in [(btn_select, "1. Select", "SELECT"), (btn_node, "2. + Node", "NODE"), (btn_beam, "3. + Beam", "BEAM")]:
        is_active = current_mode == mode_str
        b_color = COLOR_UI_BORDER if is_active else COLOR_BACKGROUND
        t_color = COLOR_HIGHLIGHT if is_active else COLOR_TEXT_MUTED
        pygame.draw.rect(screen, b_color, btn, border_radius=4)
        pygame.draw.rect(screen, COLOR_UI_BORDER, btn, width=1, border_radius=4)
        txt = font_body.render(label, True, t_color)
        screen.blit(txt, (btn.x + 12, btn.y + 12))

    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)

    if grid_enabled:
        for x in range(sim_rect.left, sim_rect.right, GRID_SIZE):
            pygame.draw.line(screen, COLOR_GRID, (x, sim_rect.top), (x, sim_rect.bottom))
        for y in range(sim_rect.top, sim_rect.bottom, GRID_SIZE):
            pygame.draw.line(screen, COLOR_GRID, (sim_rect.left, y), (sim_rect.right, y))

    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    for i, beam in enumerate(truss.beams):
        start_node = truss.nodes[beam.node_a]
        end_node = truss.nodes[beam.node_b]
        beam_color = get_stress_color(beam)
        
        if i == selected_beam_idx:
            pygame.draw.line(screen, COLOR_HIGHLIGHT, (start_node.x, start_node.y), (end_node.x, end_node.y), 8)
            
        pygame.draw.line(screen, beam_color, (start_node.x, start_node.y), (end_node.x, end_node.y), 4)

    for i, node in enumerate(truss.nodes):
        draw_force_vector(screen, node.x, node.y, node.load_x, node.load_y)

        if i == selected_node_idx or i == active_node_bnd:
            pygame.draw.circle(screen, COLOR_HIGHLIGHT, (node.x, node.y), NODE_RADIUS + 5, width=2)

        if node.is_anchor_x and node.is_anchor_y:
            pygame.draw.circle(screen, COLOR_PIN, (node.x, node.y), NODE_RADIUS + 7, width=3)
            pygame.draw.circle(screen, COLOR_PIN, (node.x, node.y), NODE_RADIUS + 2)
        elif node.is_anchor_y and not node.is_anchor_x:
            pygame.draw.line(screen, COLOR_ROLLER, (node.x - 12, node.y + 11), (node.x + 12, node.y + 11), 2)
            pygame.draw.line(screen, COLOR_ROLLER, (node.x - 8, node.y + 15), (node.x + 8, node.y + 15), 1)
            pygame.draw.circle(screen, COLOR_ROLLER, (node.x, node.y), NODE_RADIUS + 2)
        else:
            color = (234, 179, 8) if i == active_node_bnd else COLOR_NODE
            pygame.draw.circle(screen, color, (node.x, node.y), NODE_RADIUS)

    if (selected_node_idx is not None) or (selected_beam_idx is not None):
        lines = []
        header_text = ""
        
        if selected_beam_idx is not None:
            beam = truss.beams[selected_beam_idx]
            specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
            header_text = "STRUCTURAL ELEMENT"
            length_m = truss.get_beam_length(beam) / 20.0  
            stress_m_pa = beam.stress / 1e6               
            force_k_n = beam.force / 1000.0               
            
            nature = "NEUTRAL"
            if beam.stress > 1e-2:
                nature = "TENSION"
            elif beam.stress < -1e-2:
                nature = "COMPRESSION"
                
            utilization_pct = (abs(beam.stress) / specs["yield"]) * 100.0
            
            lines = [
                f"Alloy: {beam.material}",
                f"Length: {length_m:.2f} m",
                f"Force: {abs(force_k_n):.1f} kN",
                f"Stress: {abs(stress_m_pa):.1f} MPa",
                f"Type: {nature}",
                f"Load Capacity: {utilization_pct:.1f}%"
            ]
            
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = "STRUCTURAL NODE"
            
            support_str = "Free Joint"
            if node.is_anchor_x and node.is_anchor_y:
                support_str = "Pin Support"
            elif node.is_anchor_y and not node.is_anchor_x:
                support_str = "Roller Support"
                
            lines = [
                f"Type: {support_str}",
                f"Coords: ({int(node.x)}, {int(node.y)})",
                f"Load X: {node.load_x / 1000.0:.1f} kN",
                f"Load Y: {node.load_y / 1000.0:.1f} kN"
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

    active_lbl = font_body.render(f"ACTIVE MATERIAL: {MATERIAL_SPECS[truss.active_material]['label']}", True, COLOR_TEXT_MAIN)
    grid_lbl = font_body.render(f"GRID SNAP: {'ENABLED (20px)' if grid_enabled else 'DISABLED'} [G]", True, COLOR_TEXT_MUTED)
    control_lbl = font_body.render("Keys: [1] Steel [2] Aluminum [3] Titanium | [R] Clear All | Node Force: Hold [Arrows]", True, COLOR_TEXT_MUTED)
    screen.blit(active_lbl, (165, WINDOW_HEIGHT - 75))
    screen.blit(grid_lbl, (165, WINDOW_HEIGHT - 55))
    screen.blit(control_lbl, (165, WINDOW_HEIGHT - 35))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()