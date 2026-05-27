import pygame
import sys
import math
from truss_model import TrussSystem
from matrix_solver import solve_truss, calculate_benchmark_metrics

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
show_deformed = True
gravity_multiplier = 0.0
current_def_scale = 0.0
last_gravity_multiplier = 0.0
first_break_gravity = None  
show_benchmark_hud = False

input_buffer = ""
input_active = False
input_type = "Y"

fading_beams = []  

GRID_SIZE = 20
CLICK_TOLERANCE = 12
NODE_RADIUS = 6
TARGET_DEF_SCALE = 2200.0

MATERIAL_SPECS = {
    "Steel": {"yield": 250e6, "density": 7850, "label": "Structural Steel"},
    "Aluminum": {"yield": 275e6, "density": 2700, "label": "6061-T6 Aluminum"},
    "Titanium": {"yield": 880e6, "density": 4430, "label": "Ti-6Al-4V Titanium"}
}

def calculate_utilization(beam):
    if beam.is_broken or beam.force == 0.0:
        return 0.0
    specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
    yield_util = abs(beam.stress) / specs["yield"]
    if beam.stress < -1e-2:
        length_m = truss.get_beam_length(beam)
        if length_m > 0:
            p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
            buckle_util = abs(beam.force) / p_crit
            return max(yield_util, buckle_util)
    return yield_util

def get_stress_color(beam):
    if not truss.is_stable:
        return COLOR_TEXT_MUTED
    
    if beam.is_broken or beam.force == 0.0:
        return (180, 180, 185)
        
    specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
    yield_util = abs(beam.stress) / specs["yield"]
    
    is_compression = beam.stress < -1e-2
    
    if is_compression:
        length_m = truss.get_beam_length(beam)
        buckle_util = 0.0
        if length_m > 0:
            p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
            buckle_util = abs(beam.force) / p_crit
        
        utilization = max(yield_util, buckle_util)
        if utilization >= 1.0:
            return COLOR_MAX_LOAD
            
        if buckle_util > yield_util:
            if utilization > 0.82:
                pulse = (math.sin(pygame.time.get_ticks() * 0.015) + 1.0) / 2.0
                return (int(235 + pulse * 20), int(110 - pulse * 40), int(15 - pulse * 15))
            t = utilization
            r = int(160 + (239 - 160) * t)
            g = int(160 - (160 - 68) * t)
            b = int(165 - (165 - 68) * t)
            return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))
        else:
            t = utilization
            r = int(180 + (234 - 180) * t)
            g = int(180 - (180 - 179) * t)
            b = int(185 - (185 - 8) * t)
            return (max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)))
    else:
        utilization = yield_util
        if utilization >= 1.0:
            return (59, 130, 246)
        t = utilization
        r = int(180 - (180 - 34) * t)
        g = int(180 + (211 - 180) * t)
        b = int(185 + (238 - 185) * t)
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
    dx = truss.displacements[idx * 2] * current_def_scale
    dy = -truss.displacements[idx * 2 + 1] * current_def_scale
    return node.x + dx, node.y + dy

def draw_profile_preview(surf, x, y, width, height, beam):
    pygame.draw.rect(surf, (12, 12, 14), (x, y, width, height), border_radius=4)
    pygame.draw.rect(surf, COLOR_UI_BORDER, (x, y, width, height), width=1, border_radius=4)
    
    center_x = int(x + width // 2)
    center_y = int(y + height // 2)
    
    max_d = 0.32 
    scale = (width - 16) / max_d
    
    outer_dim = int(beam.dim_w * scale)
    outer_dim = max(6, min(width - 12, outer_dim))
    
    color_fill = (55, 65, 81)
    color_line = COLOR_TEXT_MUTED
    
    if beam.profile == "Square Tube":
        ox = center_x - outer_dim // 2
        oy = center_y - outer_dim // 2
        pygame.draw.rect(surf, color_fill, (ox, oy, outer_dim, outer_dim))
        pygame.draw.rect(surf, color_line, (ox, oy, outer_dim, outer_dim), width=1)
        
        inner_dim = int((beam.dim_w - 2 * beam.dim_t) * scale)
        if inner_dim > 2:
            ix = center_x - inner_dim // 2
            iy = center_y - inner_dim // 2
            pygame.draw.rect(surf, (12, 12, 14), (ix, iy, inner_dim, inner_dim))
            pygame.draw.rect(surf, color_line, (ix, iy, inner_dim, inner_dim), width=1)
            
    elif beam.profile == "H-Beam":
        w = outer_dim
        h = outer_dim
        t = max(1, int(beam.dim_t * scale))
        ox = center_x - w // 2
        oy = center_y - h // 2
        
        pygame.draw.rect(surf, color_fill, (ox, oy, w, t))
        pygame.draw.rect(surf, color_line, (ox, oy, w, t), width=1)
        
        pygame.draw.rect(surf, color_fill, (ox, oy + h - t, w, t))
        pygame.draw.rect(surf, color_line, (ox, oy + h - t, w, t), width=1)
        
        web_w = t
        web_h = h - 2 * t
        wx = center_x - web_w // 2
        wy = oy + t
        if web_h > 0:
            pygame.draw.rect(surf, color_fill, (wx, wy, web_w, web_h))
            pygame.draw.line(surf, color_line, (wx, wy), (wx, wy + web_h))
            pygame.draw.line(surf, color_line, (wx + web_w, wy), (wx + web_w, wy + web_h))
            
    elif beam.profile == "Solid Bar":
        radius = int(outer_dim // 2)
        if radius > 1:
            pygame.draw.circle(surf, color_fill, (center_x, center_y), radius)
            pygame.draw.circle(surf, color_line, (center_x, center_y), radius, width=1)
            pygame.draw.circle(surf, (12, 12, 14), (center_x, center_y), max(1, radius // 6))

is_running = True
while is_running:
    sidebar_rect = pygame.Rect(0, 0, 140, 700)
    sim_rect = pygame.Rect(160, 20, 960, 660)
    btn_select    = pygame.Rect(15, 80, 110, 35)
    btn_node      = pygame.Rect(15, 125, 110, 35)
    btn_beam      = pygame.Rect(15, 170, 110, 35)
    btn_load      = pygame.Rect(15, 215, 110, 35)
    btn_benchmark = pygame.Rect(15, 260, 110, 35)
    btn_play      = pygame.Rect(15, 430, 110, 40)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
        elif event.type == pygame.KEYDOWN:
            if input_active:
                if event.key == pygame.K_RETURN:
                    try:
                        val = float(input_buffer) * 1000.0
                        if selected_node_idx is not None:
                            if input_type == "X":
                                truss.nodes[selected_node_idx].load_x = val
                            else:
                                truss.nodes[selected_node_idx].load_y = val
                        first_break_gravity = None
                    except ValueError:
                        pass
                    input_active = False
                    input_buffer = ""
                elif event.key == pygame.K_BACKSPACE:
                    input_buffer = input_buffer[:-1]
                elif event.unicode in "0123456789.-":
                    input_buffer += event.unicode
                continue

            if event.key == pygame.K_SPACE:
                is_playing = not is_playing
                active_node_bnd = None
                input_active = False
            if event.key == pygame.K_EQUALS:
                if first_break_gravity is not None:
                    gravity_multiplier = min(first_break_gravity, gravity_multiplier + 0.5)
                else:
                    gravity_multiplier = min(100.0, gravity_multiplier + 0.5)
            elif event.key == pygame.K_MINUS:
                gravity_multiplier = max(0.0, gravity_multiplier - 0.5)
                
            if not is_playing:
                if event.key == pygame.K_r:
                    truss.clear()
                    active_node_bnd = None
                    selected_node_idx = None
                    selected_beam_idx = None
                    gravity_multiplier = 0.0
                    first_break_gravity = None
                    fading_beams = []
                    input_active = False
                    show_benchmark_hud = False
                elif event.key == pygame.K_g:
                    grid_enabled = not grid_enabled
                elif event.key == pygame.K_d:
                    show_deformed = not show_deformed
                elif event.key == pygame.K_v:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx = None
                        selected_beam_idx = None
                        active_node_bnd = None
                        first_break_gravity = None
                elif event.key == pygame.K_1:
                    truss.set_material("Steel")
                elif event.key == pygame.K_2:
                    truss.set_material("Aluminum")
                elif event.key == pygame.K_3:
                    truss.set_material("Titanium")
                elif event.key == pygame.K_p and selected_beam_idx is not None:
                    truss.beams[selected_beam_idx].cycle_profile()
                    first_break_gravity = None
                elif event.key == pygame.K_LEFTBRACKET and selected_beam_idx is not None:
                    b = truss.beams[selected_beam_idx]
                    if b.dim_w > 0.011: 
                        b.adjust_dimension(-0.005)
                        first_break_gravity = None
                elif event.key == pygame.K_RIGHTBRACKET and selected_beam_idx is not None:
                    b = truss.beams[selected_beam_idx]
                    if b.dim_w < 0.149: 
                        b.adjust_dimension(0.005)
                        first_break_gravity = None
                elif event.key == pygame.K_m and selected_beam_idx is not None:
                    b = truss.beams[selected_beam_idx]
                    m_list = ["Steel", "Aluminum", "Titanium"]
                    next_m = m_list[(m_list.index(b.material) + 1) % 3]
                    b.update_material_properties(next_m)
                    b.is_broken = False
                    b.broken_at_gravity = None
                    first_break_gravity = None
                elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                    if selected_beam_idx is not None:
                        truss.beams.pop(selected_beam_idx)
                        selected_beam_idx = None
                        first_break_gravity = None
                    elif selected_node_idx is not None:
                        idx = selected_node_idx
                        truss.beams = [b for b in truss.beams if b.node_a != idx and b.node_b != idx]
                        for b in truss.beams:
                            if b.node_a > idx: b.node_a -= 1
                            if b.node_b > idx: b.node_b -= 1
                        truss.nodes.pop(idx)
                        selected_node_idx = None
                        first_break_gravity = None

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            if sidebar_rect.collidepoint(mouse_pos):
                if btn_play.collidepoint(mouse_pos):
                    is_playing = not is_playing
                    input_active = False
                elif btn_select.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "SELECT"
                    input_active = False
                elif btn_node.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "NODE"
                    input_active = False
                elif btn_beam.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "BEAM"
                    input_active = False
                elif btn_load.collidepoint(mouse_pos) and not is_playing:
                    current_mode = "LOAD"
                    input_active = False
                elif btn_benchmark.collidepoint(mouse_pos) and not is_playing:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx = None
                        selected_beam_idx = None
                        active_node_bnd = None
                        first_break_gravity = None
                continue

            if input_active:
                input_active = False
                input_buffer = ""

            if not sim_rect.collidepoint(mouse_pos) or is_playing: continue

            clicked_node_idx = None
            for i, node in enumerate(truss.nodes):
                if pygame.math.Vector2(node.x, node.y).distance_to(mouse_pos) < CLICK_TOLERANCE:
                    clicked_node_idx = i
                    break

            if event.button == 1:
                if current_mode == "LOAD":
                    if clicked_node_idx is not None:
                        selected_node_idx = clicked_node_idx
                        selected_beam_idx = None
                elif current_mode == "SELECT":
                    selected_node_idx = clicked_node_idx
                    selected_beam_idx = None
                    if clicked_node_idx is None:
                        for i, b in enumerate(truss.beams):
                            if hasattr(b, 'is_broken') and b.is_broken: continue
                            if point_to_line_distance(mouse_pos[0], mouse_pos[1], truss.nodes[b.node_a].x, truss.nodes[b.node_a].y, truss.nodes[b.node_b].x, truss.nodes[b.node_b].y) < 8:
                                selected_beam_idx = i
                elif current_mode == "NODE" and clicked_node_idx is None:
                    truss.add_node(mouse_pos[0], mouse_pos[1], snap_enabled=grid_enabled, grid_size=GRID_SIZE)
                    first_break_gravity = None
                    show_benchmark_hud = False
                elif current_mode == "BEAM":
                    if clicked_node_idx is not None:
                        if active_node_bnd is None: active_node_bnd = clicked_node_idx
                        else:
                            truss.add_beam(active_node_bnd, clicked_node_idx)
                            active_node_bnd = None
                            first_break_gravity = None
                            show_benchmark_hud = False
                    else: active_node_bnd = None
            elif event.button == 3 and clicked_node_idx is not None:
                truss.nodes[clicked_node_idx].toggle_support()
                first_break_gravity = None
                show_benchmark_hud = False

    if selected_node_idx is not None and current_mode == "SELECT" and not is_playing and not input_active:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] or keys[pygame.K_DOWN] or keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
            first_break_gravity = None
            show_benchmark_hud = False
        if keys[pygame.K_UP]:
            truss.nodes[selected_node_idx].load_y -= 1000.0
        if keys[pygame.K_DOWN]:
            truss.nodes[selected_node_idx].load_y += 1000.0
        if keys[pygame.K_LEFT]:
            truss.nodes[selected_node_idx].load_x -= 1000.0
        if keys[pygame.K_RIGHT]:
            truss.nodes[selected_node_idx].load_x += 1000.0

    if is_playing:
        current_def_scale += (TARGET_DEF_SCALE - current_def_scale) * 0.08
        if first_break_gravity is not None and gravity_multiplier > first_break_gravity:
            gravity_multiplier = first_break_gravity

        for b in truss.beams:
            if not hasattr(b, 'is_broken'): b.is_broken = False
            if not hasattr(b, 'broken_at_gravity'): b.broken_at_gravity = None
            if b.is_broken and b.broken_at_gravity is not None:
                if gravity_multiplier < b.broken_at_gravity:
                    b.is_broken = False
                    b.broken_at_gravity = None
        
        if not any(getattr(b, 'is_broken', False) for b in truss.beams):
            first_break_gravity = None

        for node in truss.nodes:
            node.load_y += 1000.0 * gravity_multiplier
        solve_truss(truss)
        for node in truss.nodes:
            node.load_y -= 1000.0 * gravity_multiplier

        if truss.is_stable:
            for b in truss.beams:
                if not b.is_broken and calculate_utilization(b) >= 1.0:
                    b.is_broken = True
                    b.broken_at_gravity = gravity_multiplier
                    if first_break_gravity is None:
                        first_break_gravity = gravity_multiplier
                    
                    ax, ay = get_def_pos(b.node_a, truss.nodes[b.node_a])
                    bx, by = get_def_pos(b.node_b, truss.nodes[b.node_b])
                    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                    thick = max(2, min(14, int((b.area / 2.5e-3) * 4.0)))
                    
                    fading_beams.append([ax, ay, mx - 2, my, thick, 1.0, -1.0])
                    fading_beams.append([mx + 2, my, bx, by, thick, 1.0, -1.5])
                    break 
    else:
        current_def_scale += (0.0 - current_def_scale) * 0.15
        first_break_gravity = None
        for b in truss.beams:
            b.is_broken = False
            b.broken_at_gravity = None
        solve_truss(truss)

    last_gravity_multiplier = gravity_multiplier

    for fb in fading_beams[:]:
        fb[5] -= 0.04  
        fb[6] += 0.45  
        fb[1] += fb[6]
        fb[3] += fb[6]
        if fb[5] <= 0:
            fading_beams.remove(fb)

    screen.fill(COLOR_BACKGROUND)
    pygame.draw.rect(screen, COLOR_PANEL_BG, sidebar_rect)
    pygame.draw.line(screen, COLOR_UI_BORDER, (140, 0), (140, 700), 2)
    
    modes_list = [
        (btn_select, "1. Select", "SELECT"), 
        (btn_node, "2. + Node", "NODE"), 
        (btn_beam, "3. + Beam", "BEAM"), 
        (btn_load, "4. + Load", "LOAD")
    ]
    for btn, label, mode in modes_list:
        pygame.draw.rect(screen, COLOR_UI_BORDER if current_mode == mode and not is_playing else COLOR_BACKGROUND, btn, border_radius=4)
        screen.blit(font_body.render(label, True, COLOR_TEXT_MAIN), (btn.x + 12, btn.y + 10))

    pygame.draw.rect(screen, COLOR_UI_BORDER if show_benchmark_hud and not is_playing else COLOR_BACKGROUND, btn_benchmark, border_radius=4)
    screen.blit(font_body.render("5. Benchmark", True, COLOR_TEXT_MAIN), (btn_benchmark.x + 12, btn_benchmark.y + 10))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 310), (130, 310), 1)
    screen.blit(font_header.render("SYSTEM STATS", True, COLOR_TEXT_MAIN), (15, 325))
    
    total_mass = 0.0
    max_util = 0.0
    for b in truss.beams:
        if hasattr(b, 'is_broken') and b.is_broken: continue
        specs = MATERIAL_SPECS.get(b.material, MATERIAL_SPECS["Steel"])
        length_m = truss.get_beam_length(b)
        total_mass += length_m * b.area * specs["density"]
        max_util = max(max_util, calculate_utilization(b))
        
    fos_val = 1.0 / max_util if max_util > 0.0 else float('inf')
    if selected_beam_idx is not None and hasattr(truss.beams[selected_beam_idx], 'is_broken') and truss.beams[selected_beam_idx].is_broken:
        selected_beam_idx = None
    
    screen.blit(font_body.render(f"Mass: {total_mass:.1f} kg", True, COLOR_TEXT_MUTED), (15, 350))
    fos_label = font_body.render("Min FoS:", True, COLOR_TEXT_MUTED)
    screen.blit(fos_label, (15, 375))
    
    if not truss.is_stable:
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MUTED)
    elif max_util >= 1.0 or not math.isfinite(fos_val):
        fos_txt = font_header.render("FAIL", True, COLOR_LOAD)
    elif fos_val == float('inf'):
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MAIN)
    else:
        fos_txt = font_header.render(f"{fos_val:.2f}", True, COLOR_ZERO_LOAD if fos_val >= 2.0 else COLOR_MID_LOAD)
    screen.blit(fos_txt, (15 + fos_label.get_width() + 5, 373))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 410), (130, 410), 1)
    pygame.draw.rect(screen, (30, 50, 35) if is_playing else COLOR_BACKGROUND, btn_play, border_radius=4)
    pygame.draw.rect(screen, COLOR_PLAY_GREEN if is_playing else COLOR_UI_BORDER, btn_play, width=1, border_radius=4)
    screen.blit(font_header.render("|| Pause" if is_playing else "> Play", True, COLOR_PLAY_GREEN if is_playing else COLOR_TEXT_MAIN), (btn_play.x + 20, btn_play.y + 10))

    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    if grid_enabled:
        for x in range(sim_rect.left, sim_rect.right, GRID_SIZE): pygame.draw.line(screen, COLOR_GRID, (x, sim_rect.top), (x, sim_rect.bottom))
        for y in range(sim_rect.top, sim_rect.bottom, GRID_SIZE): pygame.draw.line(screen, COLOR_GRID, (sim_rect.left, y), (sim_rect.right, y))
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    node_has_connections = [False] * len(truss.nodes)
    for beam in truss.beams:
        if hasattr(beam, 'is_broken') and not beam.is_broken:
            node_has_connections[beam.node_a] = True
            node_has_connections[beam.node_b] = True

    for i, beam in enumerate(truss.beams):
        if hasattr(beam, 'is_broken') and beam.is_broken: continue
        ax, ay = get_def_pos(beam.node_a, truss.nodes[beam.node_a])
        bx, by = get_def_pos(beam.node_b, truss.nodes[beam.node_b])
        if show_deformed and truss.displacements is not None and truss.is_stable and is_playing:
            pygame.draw.line(screen, (45, 45, 50), (truss.nodes[beam.node_a].x, truss.nodes[beam.node_a].y), (truss.nodes[beam.node_b].x, truss.nodes[beam.node_b].y), 1)
        
        thickness_pixels = int(beam.dim_w * 140.0)
        thickness_pixels = max(2, min(16, thickness_pixels))
        
        if i == selected_beam_idx:
            pygame.draw.line(screen, COLOR_HIGHLIGHT, (ax, ay), (bx, by), thickness_pixels + 4)
        pygame.draw.line(screen, get_stress_color(beam), (ax, ay), (bx, by), thickness_pixels)

    for fb in fading_beams:
        alpha = int(fb[5] * 255)
        if alpha <= 0: continue
        surf = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        pygame.draw.line(surf, (220, 38, 38, alpha), (fb[0], fb[1]), (fb[2], fb[3]), fb[4])
        screen.blit(surf, (0,0))

    for i, node in enumerate(truss.nodes):
        if is_playing and not node_has_connections[i] and not node.is_anchor_x and not node.is_anchor_y:
            continue
            
        nx, ny = get_def_pos(i, node)
        
        sim_load_y = node.load_y
        if is_playing:
            sim_load_y += 1000.0 * gravity_multiplier
            
        draw_force_vector(screen, nx, ny, node.load_x, sim_load_y, COLOR_LOAD)
        if is_playing and (node.rx != 0.0 or node.ry != 0.0):
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

    if show_benchmark_hud and truss.is_stable:
        metrics = calculate_benchmark_metrics(truss)
        if metrics is not None:
            bhud_w, bhud_h = 430, 275
            bhud_x = sim_rect.left + 15
            bhud_y = sim_rect.top + 15
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (63, 63, 70, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            
            bhud_surface.blit(font_header.render("TEXTBOOK BENCHMARK METRICS", True, COLOR_TEXT_MAIN), (15, 12))
            bhud_surface.blit(font_body.render("Case: Determinant Cantilever (3-Node, 50kN Load)", True, COLOR_TEXT_MUTED), (15, 32))
            pygame.draw.line(bhud_surface, COLOR_UI_BORDER, (10, 50), (bhud_w - 10, 50), 1)
            
            rows = [
                ("Diag Force (Theory):", f"{metrics['diag_force_theory']/1000.0:.2f} kN", COLOR_TEXT_MUTED),
                (f"Diag Force (Solver):", f"{metrics['diag_force_num']/1000.0:.2f} kN  [Err: {metrics['diag_force_err']:.4f}%]", COLOR_TEXT_MAIN),
                ("Horiz Force (Theory):", f"{metrics['horiz_force_theory']/1000.0:.2f} kN", COLOR_TEXT_MUTED),
                (f"Horiz Force (Solver):", f"{metrics['horiz_force_num']/1000.0:.2f} kN  [Err: {metrics['horiz_force_err']:.4f}%]", COLOR_TEXT_MAIN),
                ("Tip Disp X (Theory):", f"{metrics['dx_theory']*1000.0:.4f} mm", COLOR_TEXT_MUTED),
                (f"Tip Disp X (Solver):", f"{metrics['dx_num']*1000.0:.4f} mm  [Err: {metrics['dx_err']:.4f}%]", COLOR_TEXT_MAIN),
                ("Tip Disp Y (Theory):", f"{metrics['dy_theory']*1000.0:.4f} mm", COLOR_TEXT_MUTED),
                (f"Tip Disp Y (Solver):", f"{metrics['dy_num']*1000.0:.4f} mm  [Err: {metrics['dy_err']:.4f}%]", COLOR_TEXT_MAIN)
            ]
            
            curr_y = 62
            for label, val_str, txt_color in rows:
                bhud_surface.blit(font_body.render(label, True, txt_color), (15, curr_y))
                
                if "Theory" in label:
                    bhud_surface.blit(font_body.render(val_str, True, COLOR_TEXT_MAIN), (170, curr_y))
                else:
                    if "[Err: 0.0000%]" in val_str or "e-1" in val_str or "e-2" in val_str:
                        err_color = COLOR_ZERO_LOAD
                    else:
                        err_color = COLOR_TEXT_MAIN
                    
                    parts = val_str.split("  ")
                    bhud_surface.blit(font_body.render(parts[0], True, COLOR_TEXT_MAIN), (170, curr_y))
                    if len(parts) > 1:
                        bhud_surface.blit(font_body.render(parts[1], True, err_color), (265, curr_y))
                        
                curr_y += 24
                
            screen.blit(bhud_surface, (bhud_x, bhud_y))
        else:
            bhud_w, bhud_h = 320, 45
            bhud_x = sim_rect.left + 15
            bhud_y = sim_rect.top + 15
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (127, 29, 29, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            bhud_surface.blit(font_body.render("BENCHMARK CASE MODIFIED OR INVALID", True, COLOR_MAX_LOAD), (15, 14))
            screen.blit(bhud_surface, (bhud_x, bhud_y))

    if ((selected_node_idx is not None) or (selected_beam_idx is not None)) and truss.is_stable:
        lines = []
        header_text = ""
        is_beam_selected = selected_beam_idx is not None
        
        if is_beam_selected:
            beam = truss.beams[selected_beam_idx]
            header_text = "STRUCTURAL ELEMENT"
            length_m = truss.get_beam_length(beam) 
            stress_m_pa = beam.stress / 1e6               
            force_k_n = beam.force / 1000.0               
            nature = "TENSION" if beam.stress > 1e-2 else ("COMPRESSION" if beam.stress < -1e-2 else "NEUTRAL")
            utilization_pct = calculate_utilization(beam) * 100.0
            
            top_lines = [
                f"Alloy: {beam.material} [M]",
                f"Force: {abs(force_k_n):.1f} kN",
                f"Stress: {abs(stress_m_pa):.1f} MPa"
            ]
            if nature == "COMPRESSION" and length_m > 0:
                p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
                top_lines.append(f"Buckling Limit: {p_crit / 1000.0:.1f} kN")
            top_lines.extend([f"Type: {nature}", f"Load Capacity: {utilization_pct:.1f}%"])
            
            geom_lines = [
                f"Profile: {beam.profile} [P]",
                f"Width/Diam: {beam.dim_w * 100.0:.1f} cm [ [ ] / [ ] ]",
                f"Thickness: {beam.dim_t * 100.0:.1f} cm",
                f"Area: {beam.area * 1e4:.1f} cm²",
                f"Length: {length_m:.2f} m"
            ]
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = "STRUCTURAL NODE"
            support_str = "Pin Support" if node.is_anchor_x and node.is_anchor_y else ("Roller Support" if node.is_anchor_y else "Free Joint")
            effective_y = node.load_y
            if is_playing:
                effective_y += 1000.0 * gravity_multiplier
            net_magnitude = math.hypot(node.load_x, effective_y) / 1000.0
            lines = [
                f"Type: {support_str}", 
                f"Coords: ({int(node.x)}, {int(node.y)})", 
                f"Net Load: {net_magnitude:.1f} kN",
                f"Load X: {node.load_x / 1000.0:.1f} kN", 
                f"Load Y: {effective_y / 1000.0:.1f} kN"
            ]
            if node.is_anchor_x or node.is_anchor_y:
                net_react = math.hypot(node.rx, node.ry) / 1000.0
                lines.append(f"Net React: {net_react:.1f} kN")
                lines.append(f"React X: {node.rx / 1000.0:.1f} kN")
                lines.append(f"React Y: {node.ry / 1000.0:.1f} kN")

        if is_beam_selected:
            hud_w = 340
            hud_h = 55 + (len(top_lines) * 24) + 12 + 95
        else:
            hud_w = max(240, max([font_body.size(line)[0] for line in lines]) + 40) if lines else 240
            hud_h = 45 + (len(lines) * 24)
            if current_mode == "LOAD" and selected_node_idx is not None:
                hud_h += 75
            
        hud_x = sim_rect.right - hud_w - 15
        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surface, (18, 18, 20, 220), (0, 0, hud_w, hud_h), border_radius=6)
        pygame.draw.rect(hud_surface, (63, 63, 70, 255), (0, 0, hud_w, hud_h), width=1, border_radius=6)
        hud_surface.blit(font_header.render(header_text, True, COLOR_TEXT_MAIN), (15, 12))

        local_y = 40
        if is_beam_selected:
            for line in top_lines:
                parts = line.split(":", 1)
                lbl_surface = font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                hud_surface.blit(lbl_surface, (15, local_y))
                hud_surface.blit(font_body.render(parts[1], True, COLOR_TEXT_MAIN), (15 + lbl_surface.get_width() + 4, local_y))
                local_y += 24
                
            local_y += 4
            pygame.draw.line(hud_surface, COLOR_UI_BORDER, (10, local_y), (hud_w - 10, local_y), 1)
            local_y += 12
            
            draw_profile_preview(hud_surface, 15, local_y, 85, 85, truss.beams[selected_beam_idx])
            
            sub_y = local_y - 2
            for line in geom_lines:
                parts = line.split(":", 1)
                lbl_surface = font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                hud_surface.blit(lbl_surface, (115, sub_y))
                hud_surface.blit(font_body.render(parts[1], True, COLOR_TEXT_MAIN), (115 + lbl_surface.get_width() + 4, sub_y))
                sub_y += 17
                
        else:
            for line in lines:
                if ":" in line:
                    parts = line.split(":", 1)
                    lbl_surface = font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                    hud_surface.blit(lbl_surface, (15, local_y))
                    hud_surface.blit(font_body.render(parts[1], True, COLOR_TEXT_MAIN), (15 + lbl_surface.get_width() + 4, local_y))
                else:
                    hud_surface.blit(font_body.render(line, True, COLOR_TEXT_MAIN), (15, local_y))
                local_y += 24
                
            if current_mode == "LOAD" and selected_node_idx is not None:
                local_y += 5
                pygame.draw.line(hud_surface, COLOR_UI_BORDER, (10, local_y), (hud_w - 10, local_y), 1)
                local_y += 10
                
                box_x = pygame.Rect(15, local_y, 95, 25)
                box_y = pygame.Rect(120, local_y, 95, 25)
                
                mx, my = pygame.mouse.get_pos()
                lx, ly = mx - hud_x, my - (sim_rect.top + 15)
                
                if pygame.mouse.get_pressed()[0]:
                    if box_x.collidepoint((lx, ly)):
                        input_active = True
                        input_type = "X"
                        input_buffer = ""
                    elif box_y.collidepoint((lx, ly)):
                        input_active = True
                        input_type = "Y"
                        input_buffer = ""
                        
                pygame.draw.rect(hud_surface, (30, 30, 35) if (input_active and input_type == "X") else (10, 10, 12), box_x, border_radius=4)
                pygame.draw.rect(hud_surface, COLOR_UI_BORDER, box_x, width=1, border_radius=4)
                hud_surface.blit(font_body.render("FX: " + (input_buffer if (input_active and input_type == "X") else f"{truss.nodes[selected_node_idx].load_x/1000.0:.1f}") + ("_" if (input_active and input_type == "X") else " kN"), True, COLOR_TEXT_MAIN), (22, local_y + 5))
                
                pygame.draw.rect(hud_surface, (30, 30, 35) if (input_active and input_type == "Y") else (10, 10, 12), box_y, border_radius=4)
                pygame.draw.rect(hud_surface, COLOR_UI_BORDER, box_y, width=1, border_radius=4)
                hud_surface.blit(font_body.render("FY: " + (input_buffer if (input_active and input_type == "Y") else f"{truss.nodes[selected_node_idx].load_y/1000.0:.1f}") + ("_" if (input_active and input_type == "Y") else " kN"), True, COLOR_TEXT_MAIN), (127, local_y + 5))
                
                hud_surface.blit(font_body.render("Click box, type value, press Enter", True, COLOR_TEXT_MUTED), (15, local_y + 35))

        screen.blit(hud_surface, (hud_x, sim_rect.top + 15))

    grav_msg = f"GRAVITY LOAD MULTIPLIER: {gravity_multiplier:.1f}x  [ - ] / [ + ]"
    if first_break_gravity is not None and math.isclose(gravity_multiplier, first_break_gravity):
        grav_msg += " (CRITICAL POINT LOCKED)"
    screen.blit(font_body.render(grav_msg, True, COLOR_TEXT_MAIN), (165, WINDOW_HEIGHT - 75))
    screen.blit(font_body.render(f"GRID SNAP: {'ENABLED (20px)' if grid_enabled else 'DISABLED'} [G] | DEFORM DISPLAY: {'ON' if show_deformed else 'OFF'} [D]", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 55))
    screen.blit(font_body.render("Keys: [1-3] Material | [R] Reset | [SPACE] Play/Pause | Arrow Keys adjust loads | [ / ] dimensions | [M] alloy | [P] structural profile | [V] Benchmark shortcut", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 35))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()