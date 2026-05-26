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
COLOR_REACTION   = (168, 85, 247)
COLOR_HIGHLIGHT  = (255, 255, 255)  
COLOR_PLAY_GREEN = (34, 197, 94)

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
is_playing = False       
grid_enabled = True
show_deformed = False
gravity_multiplier = 1.0
GRID_SIZE = 20
CLICK_TOLERANCE = 12
NODE_RADIUS = 6
DEF_SCALE = 500.0

MATERIAL_SPECS = {
    "Steel": {"yield": 250e6, "density": 7850, "label": "Structural Steel"},
    "Aluminum": {"yield": 275e6, "density": 2700, "label": "6061-T6 Aluminum"},
    "Titanium": {"yield": 880e6, "density": 4430, "label": "Ti-6Al-4V Titanium"}
}

def calculate_utilization(beam):
    if beam.force == 0.0:
        return 0.0
    specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
    yield_util = abs(beam.stress) / specs["yield"]
    if beam.stress < -1e-2:
        length_m = truss.get_beam_length(beam) / 20.0
        if length_m > 0:
            p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
            buckle_util = abs(beam.force) / p_crit
            return max(yield_util, buckle_util)
    return yield_util

def get_stress_color(beam):
    if not truss.is_stable:
        return COLOR_TEXT_MUTED
    utilization = calculate_utilization(beam)
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
        g = int(COLOR_MID_LOAD[1] + (COLOR_MAX_LOAD[1] - COLOR_MAX_LOAD[1]) * t)
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

def draw_force_vector(surface, cx, cy, fx, fy, color=COLOR_LOAD):
    mag = math.hypot(fx, fy)
    if mag < 1e-1: return
    dx = fx / mag
    dy = fy / mag
    arrow_len = max(20, min(65, int(mag / 1000.0) * 1.5 + 20))
    start_x = cx - dx * arrow_len
    start_y = cy - dy * arrow_len
    pygame.draw.line(surface, color, (start_x, start_y), (cx, cy), 3)
    wing_len = 8
    angle = math.atan2(cy - start_y, cx - start_x)
    pygame.draw.polygon(surface, color, [(cx, cy), (cx - wing_len * math.cos(angle + 0.4), cy - wing_len * math.sin(angle + 0.4)), (cx - wing_len * math.cos(angle - 0.4), cy - wing_len * math.sin(angle - 0.4))])

def get_def_pos(idx, node):
    if truss.displacements is None or not show_deformed or not truss.is_stable:
        return node.x, node.y
    dx = truss.displacements[idx * 2] * DEF_SCALE
    dy = -truss.displacements[idx * 2 + 1] * DEF_SCALE
    return node.x + dx, node.y + dy

is_running = True
while is_running:
    sidebar_rect = pygame.Rect(0, 0, 140, 700)
    sim_rect = pygame.Rect(160, 20, 960, 660)
    btn_select = pygame.Rect(15, 80, 110, 40)
    btn_node   = pygame.Rect(15, 135, 110, 40)
    btn_beam   = pygame.Rect(15, 190, 110, 40)
    btn_play   = pygame.Rect(15, 385, 110, 40)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                is_playing = not is_playing
                gravity_multiplier = 1.0
                active_node_bnd = None
            if not is_playing:
                if event.key == pygame.K_r:
                    truss.clear()
                    active_node_bnd = None
                    selected_node_idx = None
                    selected_beam_idx = None
                elif event.key == pygame.K_g:
                    grid_enabled = not grid_enabled
                elif event.key == pygame.K_d:
                    show_deformed = not show_deformed
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
                            if b.node_a > idx: b.node_a -= 1
                            if b.node_b > idx: b.node_b -= 1
                        truss.nodes.pop(idx)
                        selected_node_idx = None

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            if sidebar_rect.collidepoint(mouse_pos):
                if btn_play.collidepoint(mouse_pos):
                    is_playing = not is_playing
                    gravity_multiplier = 1.0
                elif btn_select.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "SELECT"
                elif btn_node.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "NODE"
                elif btn_beam.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "BEAM"
                continue

            if not sim_rect.collidepoint(mouse_pos) or is_playing: continue

            clicked_node_idx = None
            for i, node in enumerate(truss.nodes):
                if pygame.math.Vector2(node.x, node.y).distance_to(mouse_pos) < CLICK_TOLERANCE:
                    clicked_node_idx = i
                    break

            if event.button == 1:
                keys = pygame.key.get_pressed()
                if keys[pygame.K_l] and clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].load_y += 10000.0
                    active_node_bnd = None
                else:
                    if current_mode == "SELECT":
                        selected_node_idx = clicked_node_idx
                        selected_beam_idx = None
                        if clicked_node_idx is None:
                            for i, b in enumerate(truss.beams):
                                if point_to_line_distance(mouse_pos[0], mouse_pos[1], truss.nodes[b.node_a].x, truss.nodes[b.node_a].y, truss.nodes[b.node_b].x, truss.nodes[b.node_b].y) < 8:
                                    selected_beam_idx = i
                    elif current_mode == "NODE" and clicked_node_idx is None:
                        truss.add_node(mouse_pos[0], mouse_pos[1], snap_enabled=grid_enabled, grid_size=GRID_SIZE)
                    elif current_mode == "BEAM":
                        if clicked_node_idx is not None:
                            if active_node_bnd is None: active_node_bnd = clicked_node_idx
                            else:
                                truss.add_beam(active_node_bnd, clicked_node_idx)
                                active_node_bnd = None
                        else: active_node_bnd = None
            elif event.button == 3 and clicked_node_idx is not None:
                truss.nodes[clicked_node_idx].toggle_support()

    if selected_node_idx is not None and current_mode == "SELECT" and not is_playing:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            truss.nodes[selected_node_idx].load_y -= 1000.0
        if keys[pygame.K_DOWN]:
            truss.nodes[selected_node_idx].load_y += 1000.0
        if keys[pygame.K_LEFT]:
            truss.nodes[selected_node_idx].load_x -= 1000.0
        if keys[pygame.K_RIGHT]:
            truss.nodes[selected_node_idx].load_x += 1000.0

    if is_playing:
        gravity_multiplier += 0.05
        for node in truss.nodes:
            node.load_y += 500.0 * gravity_multiplier
        solve_truss(truss)
        for node in truss.nodes:
            node.load_y -= 500.0 * gravity_multiplier
        for b in truss.beams:
            if calculate_utilization(b) > 1.0:
                truss.beams.remove(b)
    else:
        solve_truss(truss)

    screen.fill(COLOR_BACKGROUND)
    pygame.draw.rect(screen, COLOR_PANEL_BG, sidebar_rect)
    pygame.draw.line(screen, COLOR_UI_BORDER, (140, 0), (140, 700), 2)
    
    for btn, label, mode in [(btn_select, "1. Select", "SELECT"), (btn_node, "2. + Node", "NODE"), (btn_beam, "3. + Beam", "BEAM")]:
        pygame.draw.rect(screen, COLOR_UI_BORDER if current_mode == mode and not is_playing else COLOR_BACKGROUND, btn, border_radius=4)
        screen.blit(font_body.render(label, True, COLOR_TEXT_MAIN), (btn.x + 12, btn.y + 12))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 260), (130, 260), 1)
    screen.blit(font_header.render("SYSTEM STATS", True, COLOR_TEXT_MAIN), (15, 280))
    
    total_mass = 0.0
    max_util = 0.0
    for b in truss.beams:
        specs = MATERIAL_SPECS.get(b.material, MATERIAL_SPECS["Steel"])
        length_m = truss.get_beam_length(b) / 20.0
        total_mass += length_m * b.area * specs["density"]
        max_util = max(max_util, calculate_utilization(b))
        
    fos_val = 1.0 / max_util if max_util > 0.0 else float('inf')
    
    screen.blit(font_body.render(f"Mass: {total_mass:.1f} kg", True, COLOR_TEXT_MUTED), (15, 310))
    fos_label = font_body.render("Min FoS:", True, COLOR_TEXT_MUTED)
    screen.blit(fos_label, (15, 335))
    
    if not truss.is_stable:
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MUTED)
    elif max_util >= 1.0:
        fos_txt = font_header.render("FAIL", True, COLOR_LOAD)
    elif fos_val == float('inf'):
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MAIN)
    else:
        fos_txt = font_header.render(f"{fos_val:.2f}", True, COLOR_ZERO_LOAD if fos_val >= 2.0 else COLOR_MID_LOAD)
    screen.blit(fos_txt, (15 + fos_label.get_width() + 5, 333))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 365), (130, 365), 1)
    pygame.draw.rect(screen, (30, 50, 35) if is_playing else COLOR_BACKGROUND, btn_play, border_radius=4)
    pygame.draw.rect(screen, COLOR_PLAY_GREEN if is_playing else COLOR_UI_BORDER, btn_play, width=1, border_radius=4)
    screen.blit(font_header.render("|| Pause" if is_playing else "> Play", True, COLOR_PLAY_GREEN if is_playing else COLOR_TEXT_MAIN), (btn_play.x + 20, btn_play.y + 10))

    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    if grid_enabled:
        for x in range(sim_rect.left, sim_rect.right, GRID_SIZE): pygame.draw.line(screen, COLOR_GRID, (x, sim_rect.top), (x, sim_rect.bottom))
        for y in range(sim_rect.top, sim_rect.bottom, GRID_SIZE): pygame.draw.line(screen, COLOR_GRID, (sim_rect.left, y), (sim_rect.right, y))
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    for i, beam in enumerate(truss.beams):
        ax, ay = get_def_pos(beam.node_a, truss.nodes[beam.node_a])
        bx, by = get_def_pos(beam.node_b, truss.nodes[beam.node_b])
        if show_deformed and truss.displacements is not None and truss.is_stable:
            pygame.draw.line(screen, (45, 45, 50), (truss.nodes[beam.node_a].x, truss.nodes[beam.node_a].y), (truss.nodes[beam.node_b].x, truss.nodes[beam.node_b].y), 1)
        if i == selected_beam_idx:
            pygame.draw.line(screen, COLOR_HIGHLIGHT, (ax, ay), (bx, by), 8)
        pygame.draw.line(screen, get_stress_color(beam), (ax, ay), (bx, by), 4)

    for i, node in enumerate(truss.nodes):
        nx, ny = get_def_pos(i, node)
        draw_force_vector(screen, nx, ny, node.load_x, node.load_y, COLOR_LOAD)
        if node.rx != 0.0 or node.ry != 0.0:
            draw_force_vector(screen, nx, ny, node.rx, node.ry, COLOR_REACTION)
        if i == selected_node_idx or i == active_node_bnd:
            pygame.draw.circle(screen, COLOR_HIGHLIGHT, (nx, ny), NODE_RADIUS + 5, width=2)
        if node.is_anchor_x and node.is_anchor_y:
            pygame.draw.circle(screen, COLOR_PIN, (nx, ny), NODE_RADIUS + 7, width=3)
            pygame.draw.circle(screen, COLOR_PIN, (nx, ny), NODE_RADIUS + 2)
        elif node.is_anchor_y and not node.is_anchor_x:
            pygame.draw.line(screen, COLOR_ROLLER, (nx - 12, ny + 11), (nx + 12, ny + 11), 2)
            pygame.draw.line(screen, COLOR_ROLLER, (nx - 8, ny + 15), (nx + 8, ny + 15), 1)
            pygame.draw.circle(screen, COLOR_ROLLER, (nx, ny), NODE_RADIUS + 2)
        else:
            pygame.draw.circle(screen, (234, 179, 8) if i == active_node_bnd else COLOR_NODE, (nx, ny), NODE_RADIUS)

    if not truss.is_stable:
        txt = "STRUCTURE UNSTABLE / MECHANISM DETECTED"
        w = font_header.size(txt)[0] + 30
        alert = pygame.Rect(sim_rect.left + (sim_rect.width - w) // 2, sim_rect.top + 20, w, 45)
        pygame.draw.rect(screen, (127, 29, 29, 230), alert, border_radius=6)
        pygame.draw.rect(screen, COLOR_LOAD, alert, width=1, border_radius=6)
        screen.blit(font_header.render(txt, True, COLOR_HIGHLIGHT), (alert.x + 15, alert.y + 14))

    if ((selected_node_idx is not None) or (selected_beam_idx is not None)) and truss.is_stable:
        lines = []
        header_text = ""
        if selected_beam_idx is not None:
            beam = truss.beams[selected_beam_idx]
            header_text = "STRUCTURAL ELEMENT"
            length_m = truss.get_beam_length(beam) / 20.0  
            stress_m_pa = beam.stress / 1e6               
            force_k_n = beam.force / 1000.0               
            nature = "TENSION" if beam.stress > 1e-2 else ("COMPRESSION" if beam.stress < -1e-2 else "NEUTRAL")
            utilization_pct = calculate_utilization(beam) * 100.0
            lines = [f"Alloy: {beam.material}", f"Length: {length_m:.2f} m", f"Force: {abs(force_k_n):.1f} kN", f"Stress: {abs(stress_m_pa):.1f} MPa", f"Type: {nature}", f"Load Capacity: {utilization_pct:.1f}%"]
            if nature == "COMPRESSION" and length_m > 0:
                p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
                lines.insert(3, f"Buckling Limit: {p_crit / 1000.0:.1f} kN")
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = "STRUCTURAL NODE"
            support_str = "Pin Support" if node.is_anchor_x and node.is_anchor_y else ("Roller Support" if node.is_anchor_y else "Free Joint")
            lines = [f"Type: {support_str}", f"Coords: ({int(node.x)}, {int(node.y)})", f"Load X: {node.load_x / 1000.0:.1f} kN", f"Load Y: {node.load_y / 1000.0:.1f} kN"]
            if node.is_anchor_x or node.is_anchor_y:
                lines.append(f"React X: {node.rx / 1000.0:.1f} kN")
                lines.append(f"React Y: {node.ry / 1000.0:.1f} kN")

        hud_w = max(240, max([font_body.size(line)[0] for line in lines]) + 40) if lines else 240
        hud_h = 45 + (len(lines) * 24)
        hud_x = sim_rect.right - hud_w - 15
        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surface, (18, 18, 20, 220), (0, 0, hud_w, hud_h), border_radius=6)
        pygame.draw.rect(hud_surface, (63, 63, 70, 255), (0, 0, hud_w, hud_h), width=1, border_radius=6)
        hud_surface.blit(font_header.render(header_text, True, COLOR_TEXT_MAIN), (15, 12))

        local_y = 40
        for line in lines:
            if ":" in line:
                parts = line.split(":", 1)
                lbl_surface = font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                hud_surface.blit(lbl_surface, (15, local_y))
                hud_surface.blit(font_body.render(parts[1], True, COLOR_TEXT_MAIN), (15 + lbl_surface.get_width() + 4, local_y))
            else:
                hud_surface.blit(font_body.render(line, True, COLOR_TEXT_MAIN), (15, local_y))
            local_y += 24
        screen.blit(hud_surface, (hud_x, sim_rect.top + 15))

    screen.blit(font_body.render(f"ACTIVE MATERIAL: {MATERIAL_SPECS[truss.active_material]['label']}", True, COLOR_TEXT_MAIN), (165, WINDOW_HEIGHT - 75))
    screen.blit(font_body.render(f"GRID SNAP: {'ENABLED (20px)' if grid_enabled else 'DISABLED'} [G] | DEFORM DISPLAY: {'ON' if show_deformed else 'OFF'} [D]", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 55))
    screen.blit(font_body.render("Keys: [1] Steel [2] Aluminum [3] Titanium | [R] Clear All | [SPACE] Play/Pause Simulation | Arrow Keys adjust select loads", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 35))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()