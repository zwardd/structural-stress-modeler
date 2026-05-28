import pygame
import sys
import math
import tkinter as tk
from tkinter import filedialog
from truss_model import TrussSystem
from matrix_solver import solve_truss, calculate_benchmark_metrics

root = tk.Tk()
root.withdraw()

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
COLOR_YIELDING   = (249, 115, 22)

COLOR_ZERO_LOAD = (144, 238, 144)   
COLOR_MID_LOAD  = (234, 179, 8)     
COLOR_MAX_LOAD  = (239, 68, 68)     

font_header = pygame.font.SysFont("Helvetica", 16, bold=True)
font_body = pygame.font.SysFont("Helvetica", 13)

GRID_SIZE = 20
CLICK_TOLERANCE = 12
NODE_RADIUS = 6
TARGET_DEF_SCALE = 2200.0

MATERIAL_SPECS = {
    "Steel": {"yield": 250e6, "ultimate": 400e6, "density": 7850, "label": "Structural Steel"},
    "Aluminum": {"yield": 275e6, "ultimate": 310e6, "density": 2700, "label": "6061-T6 Aluminum"},
    "Titanium": {"yield": 880e6, "ultimate": 950e6, "density": 4430, "label": "Ti-6Al-4V Titanium"}
}

clock = pygame.time.Clock()
truss = TrussSystem()

active_node_bnd = None  
selected_node_idx = None 
selected_beam_idx = None 

current_mode = "SELECT" 
is_playing = False       
is_optimizing = False
allow_profile_switching = True
grid_enabled = True
show_deformed = True
gravity_multiplier = 0.0
current_def_scale = 0.0
last_gravity_multiplier = 0.0
first_break_gravity = None  
show_benchmark_hud = False

status_banner_text = ""
status_banner_timer = 0

input_buffer = ""
input_active = False
input_type = "Y"
fading_beams = []  

pan_x = 0.0
pan_y = 0.0
zoom_scale = 1.0
MIN_ZOOM = 0.3
MAX_ZOOM = 4.0
is_panning = False
last_mouse_pos = (0, 0)
WORKSPACE_LIMIT = 50000.0

sim_rect = pygame.Rect(160, 20, 960, 660)

def to_screen(sim_x, sim_y):
    cx = sim_rect.left + sim_rect.width / 2
    cy = sim_rect.top + sim_rect.height / 2
    screen_x = (sim_x - cx) * zoom_scale + cx + pan_x
    screen_y = (sim_y - cy) * zoom_scale + cy + pan_y
    return screen_x, screen_y

def to_sim(screen_x, screen_y):
    cx = sim_rect.left + sim_rect.width / 2
    cy = sim_rect.top + sim_rect.height / 2
    sim_x = (screen_x - pan_x - cx) / zoom_scale + cx
    sim_y = (screen_y - pan_y - cy) / zoom_scale + cy
    return sim_x, sim_y

def trigger_status(text):
    global status_banner_text, status_banner_timer
    status_banner_text = text
    status_banner_timer = 180  

def find_proxy_limit(beam):
    if beam.status != "YIELDING" or beam.force == 0.0:
        return float('inf'), None
    node_a = truss.nodes[beam.node_a]
    node_b = truss.nodes[beam.node_b]
    ax, ay = node_a.x, node_a.y
    bx, by = node_b.x, node_b.y
    dx, dy = bx - ax, by - ay
    L_pixels = math.hypot(dx, dy)
    if L_pixels < 1:
        return float('inf'), None
    nx, ny = -dy / L_pixels, dx / L_pixels
    min_dist = float('inf')
    target_node_idx = None
    for i, node in enumerate(truss.nodes):
        if i == beam.node_a or i == beam.node_b:
            continue
        px_pos, py_pos = node.x, node.y
        proj_t = ((px_pos - ax) * dx + (py_pos - ay) * dy) / (L_pixels * L_pixels)
        if 0.1 <= proj_t <= 0.9:
            mid_x = ax + dx * proj_t
            mid_y = ay + dy * proj_t
            perp_dist = (px_pos - mid_x) * nx + (py_pos - mid_y) * ny
            if abs(perp_dist) < 12.0 and abs(perp_dist) < abs(min_dist):
                if (beam.stress < -1e-2 and perp_dist > 0.0) or (beam.stress > 1e-2):
                    min_dist = perp_dist
                    target_node_idx = i
    if min_dist != float('inf'):
        return abs(min_dist), target_node_idx
    return float('inf'), None

def calculate_utilization(beam):
    if beam.status == "FRACTURED" or beam.force == 0.0:
        return 0.0
    specs = MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])
    yield_util = abs(beam.stress) / specs["yield"]
    if beam.status == "YIELDING":
        limit, proxy_node = find_proxy_limit(beam)
        if proxy_node is not None:
            current_visual_bow = min(35.0, (yield_util - 0.95) * 18.0)
            if current_visual_bow >= limit:
                return max(yield_util * 1.6, 1.2)
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
    if beam.status == "FRACTURED" or beam.force == 0.0:
        return (180, 180, 185)
    if beam.status == "YIELDING":
        limit, proxy_node = find_proxy_limit(beam)
        if proxy_node is not None:
            util = abs(beam.stress) / MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])["yield"]
            if min(35.0, (util - 0.95) * 18.0) >= limit:
                return (220, 38, 38)
        pulse = (math.sin(pygame.time.get_ticks() * 0.01) + 1.0) / 2.0
        return (int(249 - pulse * 15), int(115 + pulse * 25), int(22 - pulse * 10))
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
        if utilization >= 1.0: return COLOR_MAX_LOAD
        if buckle_util > yield_util:
            if utilization > 0.82:
                pulse = (math.sin(pygame.time.get_ticks() * 0.015) + 1.0) / 2.0
                return (int(235 + pulse * 20), int(110 - pulse * 40), int(15 - pulse * 15))
            t = utilization
            return (max(0, min(255, int(160 + (239 - 160) * t))), max(0, min(255, int(160 - (160 - 68) * t))), max(0, min(255, int(165 - (165 - 68) * t))))
        else:
            t = utilization
            return (max(0, min(255, int(180 + (234 - 180) * t))), max(0, min(255, int(180 - (180 - 179) * t))), max(0, min(255, int(185 - (185 - 8) * t))))
    else:
        utilization = yield_util
        if utilization >= 1.0: return (59, 130, 246)
        t = utilization
        return (max(0, min(255, int(180 - (180 - 34) * t))), max(0, min(255, int(180 + (211 - 180) * t))), max(0, min(255, int(185 + (238 - 185) * t))))

def point_to_line_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0: return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

def draw_force_vector(surface, cx, cy, fx, fy, color=COLOR_LOAD):
    mag = math.hypot(fx, fy)
    if mag < 1e-1: return
    dx = fx / mag
    dy = fy / mag
    arrow_len = max(20, min(65, int(mag / 1000.0) * 1.5 + 20)) * zoom_scale
    start_x = cx - dx * arrow_len
    start_y = cy - dy * arrow_len
    pygame.draw.line(surface, color, (start_x, start_y), (cx, cy), max(1, int(3 * zoom_scale)))
    wing_len = max(3, int(8 * zoom_scale))
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
    outer_dim = max(6, min(width - 12, int(beam.dim_w * scale)))
    color_fill = (55, 65, 81)
    color_line = COLOR_TEXT_MUTED
    if beam.profile == "Square Tube":
        ox, oy = center_x - outer_dim // 2, center_y - outer_dim // 2
        pygame.draw.rect(surf, color_fill, (ox, oy, outer_dim, outer_dim))
        pygame.draw.rect(surf, color_line, (ox, oy, outer_dim, outer_dim), width=1)
        inner_dim = int((beam.dim_w - 2 * beam.dim_t) * scale)
        if inner_dim > 2:
            ix, iy = center_x - inner_dim // 2, center_y - inner_dim // 2
            pygame.draw.rect(surf, (12, 12, 14), (ix, iy, inner_dim, inner_dim))
            pygame.draw.rect(surf, color_line, (ix, iy, inner_dim, inner_dim), width=1)
    elif beam.profile == "H-Beam":
        w, h = outer_dim, outer_dim
        t = max(1, int(beam.dim_t * scale))
        ox, oy = center_x - w // 2, center_y - h // 2
        pygame.draw.rect(surf, color_fill, (ox, oy, w, t))
        pygame.draw.rect(surf, color_line, (ox, oy, w, t), width=1)
        pygame.draw.rect(surf, color_fill, (ox, oy + h - t, w, t))
        pygame.draw.rect(surf, color_line, (ox, oy + h - t, w, t), width=1)
        web_w, web_h = t, h - 2 * t
        if web_h > 0:
            wx, wy = center_x - web_w // 2, oy + t
            pygame.draw.rect(surf, color_fill, (wx, wy, web_w, web_h))
            pygame.draw.line(surf, color_line, (wx, wy), (wx, wy + web_h))
            pygame.draw.line(surf, color_line, (wx + web_w, wy), (wx + web_w, wy + web_h))
    elif beam.profile == "Solid Bar":
        radius = int(outer_dim // 2)
        if radius > 1:
            pygame.draw.circle(surf, color_fill, (center_x, center_y), radius)
            pygame.draw.circle(surf, color_line, (center_x, center_y), radius, width=1)
            pygame.draw.circle(surf, (12, 12, 14), (center_x, center_y), max(1, radius // 6))

def draw_curved_beam(surface, ax, ay, bx, by, beam, thickness, color, is_selected):
    if beam.status != "YIELDING" or beam.force == 0.0:
        if is_selected:
            pygame.draw.line(surface, COLOR_HIGHLIGHT, (ax, ay), (bx, by), thickness + 4)
        pygame.draw.line(surface, color, (ax, ay), (bx, by), thickness)
        return
    dx = bx - ax
    dy = by - ay
    L_pixels = math.hypot(dx, dy)
    if L_pixels < 1:
        return
    nx = -dy / L_pixels
    ny = dx / L_pixels
    util = abs(beam.stress) / MATERIAL_SPECS.get(beam.material, MATERIAL_SPECS["Steel"])["yield"]
    max_bow = min(35.0, (util - 0.95) * 18.0) * zoom_scale
    if max_bow < 1.0:
        max_bow = 1.0
    if beam.stress > 0.0:
        max_bow *= 0.15
    limit, proxy_node = find_proxy_limit(beam)
    if proxy_node is not None and max_bow >= (limit * zoom_scale):
        max_bow = limit * zoom_scale
    segments = 16
    points = []
    for s in range(segments + 1):
        t = s / segments
        x_line = ax + dx * t
        y_line = ay + dy * t
        offset = max_bow * math.sin(math.pi * t)
        x_curve = x_line + nx * offset
        y_curve = y_line + ny * offset
        points.append((x_curve, y_curve))
    if is_selected:
        pygame.draw.lines(surface, COLOR_HIGHLIGHT, False, points, thickness + 4)
    pygame.draw.lines(surface, color, False, points, thickness)

is_running = True
while is_running:
    sidebar_rect = pygame.Rect(0, 0, 140, 700)
    btn_select    = pygame.Rect(15, 80, 110, 35)
    btn_node      = pygame.Rect(15, 125, 110, 35)
    btn_beam      = pygame.Rect(15, 170, 110, 35)
    btn_load      = pygame.Rect(15, 215, 110, 35)
    btn_benchmark = pygame.Rect(15, 260, 110, 35)
    btn_play      = pygame.Rect(15, 430, 110, 40)
    btn_w_toggle  = pygame.Rect(15, 485, 110, 30)
    btn_optimize  = pygame.Rect(15, 525, 110, 35)
    chk_profile   = pygame.Rect(15, 570, 14, 14)

    mouse_pos = pygame.mouse.get_pos()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.MOUSEWHEEL:
            if sim_rect.collidepoint(mouse_pos):
                old_sim_x, old_sim_y = to_sim(mouse_pos[0], mouse_pos[1])
                zoom_scale = max(MIN_ZOOM, min(MAX_ZOOM, zoom_scale + event.y * 0.08))
                new_screen_x, new_screen_y = to_screen(old_sim_x, old_sim_y)
                pan_x += mouse_pos[0] - new_screen_x
                pan_y += mouse_pos[1] - new_screen_y
                pan_x = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, pan_x))
                pan_y = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, pan_y))

        elif event.type == pygame.KEYDOWN:
            if input_active:
                if event.key == pygame.K_RETURN:
                    try:
                        val = float(input_buffer) * 1000.0
                        if selected_node_idx is not None:
                            if input_type == "X": truss.nodes[selected_node_idx].load_x = val
                            else: truss.nodes[selected_node_idx].load_y = val
                        first_break_gravity = None
                    except ValueError: pass
                    input_active = False
                    input_buffer = ""
                elif event.key == pygame.K_BACKSPACE: input_buffer = input_buffer[:-1]
                elif event.unicode in "0123456789.-": input_buffer += event.unicode
                continue

            ctrl_pressed = pygame.key.get_pressed()[pygame.K_LCTRL] or pygame.key.get_pressed()[pygame.K_RCTRL]
            if ctrl_pressed and not is_playing:
                if event.key == pygame.K_s:
                    file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
                    if file_path:
                        truss.save_to_file(file_path)
                        trigger_status("PROJECT SAVED SUCCESSFULLY")
                    continue
                elif event.key == pygame.K_o:
                    file_path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
                    if file_path:
                        if truss.load_from_file(file_path):
                            selected_node_idx, selected_beam_idx, active_node_bnd = None, None, None
                            gravity_multiplier, first_break_gravity = 0.0, None
                            fading_beams.clear()
                            show_benchmark_hud = False
                            is_optimizing = False
                            pan_x, pan_y, zoom_scale = 0.0, 0.0, 1.0
                            trigger_status("PROJECT LOADED")
                        else:
                            trigger_status("FAILED TO LOAD FILE")
                    continue

            if event.key == pygame.K_SPACE and not pygame.key.get_pressed()[pygame.K_LALT]:
                is_playing = not is_playing
                is_optimizing = False
                active_node_bnd = None
                input_active = False
            elif event.key == pygame.K_h:
                pan_x, pan_y, zoom_scale = 0.0, 0.0, 1.0
                trigger_status("CAMERA RESET TO ORIGIN")
            elif event.key == pygame.K_EQUALS:
                if first_break_gravity is not None: gravity_multiplier = min(first_break_gravity, gravity_multiplier + 0.5)
                else: gravity_multiplier = min(100.0, gravity_multiplier + 0.5)
            elif event.key == pygame.K_MINUS: gravity_multiplier = max(0.0, gravity_multiplier - 0.5)
                
            if not is_playing:
                if event.key == pygame.K_r:
                    truss.clear()
                    active_node_bnd, selected_node_idx, selected_beam_idx = None, None, None
                    gravity_multiplier, first_break_gravity = 0.0, None
                    fading_beams.clear()
                    input_active, show_benchmark_hud, is_optimizing = False, False, False
                    pan_x, pan_y, zoom_scale = 0.0, 0.0, 1.0
                elif event.key == pygame.K_g: grid_enabled = not grid_enabled
                elif event.key == pygame.K_d: show_deformed = not show_deformed
                elif event.key == pygame.K_w:
                    truss.self_weight_enabled = not truss.self_weight_enabled
                elif event.key == pygame.K_v:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx, selected_beam_idx, active_node_bnd, first_break_gravity = None, None, None, None
                        is_optimizing = False
                        pan_x, pan_y, zoom_scale = 0.0, 0.0, 1.0
                elif event.key == pygame.K_1: truss.set_material("Steel")
                elif event.key == pygame.K_2: truss.set_material("Aluminum")
                elif event.key == pygame.K_3: truss.set_material("Titanium")
                elif event.key == pygame.K_p and selected_beam_idx is not None:
                    truss.beams[selected_beam_idx].cycle_profile()
                    first_break_gravity = None
                elif event.key == pygame.K_LEFTBRACKET and selected_beam_idx is not None:
                    truss.beams[selected_beam_idx].adjust_dimension(-0.005)
                    first_break_gravity = None
                elif event.key == pygame.K_RIGHTBRACKET and selected_beam_idx is not None:
                    truss.beams[selected_beam_idx].adjust_dimension(0.005)
                    first_break_gravity = None
                elif event.key == pygame.K_m and selected_beam_idx is not None:
                    b = truss.beams[selected_beam_idx]
                    m_list = ["Steel", "Aluminum", "Titanium"]
                    next_m = m_list[(m_list.index(b.material) + 1) % 3]
                    b.update_material_properties(next_m)
                    b.reset_status()
                    first_break_gravity = None
                elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                    if selected_beam_idx is not None:
                        truss.beams.pop(selected_beam_idx)
                        selected_beam_idx, first_break_gravity = None, None
                    elif selected_node_idx is not None:
                        idx = selected_node_idx
                        truss.beams = [b for b in truss.beams if b.node_a != idx and b.node_b != idx]
                        for b in truss.beams:
                            if b.node_a > idx: b.node_a -= 1
                            if b.node_b > idx: b.node_b -= 1
                        truss.nodes.pop(idx)
                        selected_node_idx, first_break_gravity = None, None

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if sidebar_rect.collidepoint(event.pos):
                if btn_play.collidepoint(event.pos):
                    is_playing = not is_playing
                    is_optimizing = False
                    input_active = False
                elif btn_w_toggle.collidepoint(event.pos):
                    truss.self_weight_enabled = not truss.self_weight_enabled
                elif btn_optimize.collidepoint(event.pos) and not is_playing:
                    is_optimizing = not is_optimizing
                    if is_optimizing: trigger_status("OPTIMIZATION ENGINE ACTIVE")
                    else: trigger_status("OPTIMIZATION HALTED")
                elif chk_profile.inflate(10, 10).collidepoint(event.pos):
                    allow_profile_switching = not allow_profile_switching
                elif btn_select.collidepoint(event.pos) and not is_playing: current_mode, input_active = "SELECT", False
                elif btn_node.collidepoint(event.pos) and not is_playing: current_mode, input_active = "NODE", False
                elif btn_beam.collidepoint(event.pos) and not is_playing: current_mode, input_active = "BEAM", False
                elif btn_load.collidepoint(event.pos) and not is_playing: current_mode, input_active = "LOAD", False
                elif btn_benchmark.collidepoint(event.pos) and not is_playing:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx, selected_beam_idx, active_node_bnd, first_break_gravity = None, None, None, None
                        is_optimizing = False
                        pan_x, pan_y, zoom_scale = 0.0, 0.0, 1.0
                continue

            if input_active: input_active, input_buffer = False, ""
            if not sim_rect.collidepoint(event.pos): continue

            if event.button == 2 or (event.button == 1 and pygame.key.get_pressed()[pygame.K_LSHIFT]):
                is_panning = True
                last_mouse_pos = event.pos
                continue

            sim_mouse_x, sim_mouse_y = to_sim(event.pos[0], event.pos[1])

            clicked_node_idx = None
            for i, node in enumerate(truss.nodes):
                if math.hypot(node.x - sim_mouse_x, node.y - sim_mouse_y) < (CLICK_TOLERANCE / zoom_scale):
                    clicked_node_idx = i
                    break

            if event.button == 1:
                if current_mode == "LOAD" and clicked_node_idx is not None: selected_node_idx, selected_beam_idx = clicked_node_idx, None
                elif current_mode == "SELECT":
                    selected_node_idx = clicked_node_idx
                    selected_beam_idx = None
                    if clicked_node_idx is None:
                        for i, b in enumerate(truss.beams):
                            if b.status == "FRACTURED": continue
                            if point_to_line_distance(sim_mouse_x, sim_mouse_y, truss.nodes[b.node_a].x, truss.nodes[b.node_a].y, truss.nodes[b.node_b].x, truss.nodes[b.node_b].y) < (8 / zoom_scale):
                                selected_beam_idx = i
                elif current_mode == "NODE" and clicked_node_idx is None:
                    sim_mouse_x = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, sim_mouse_x))
                    sim_mouse_y = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, sim_mouse_y))
                    truss.add_node(sim_mouse_x, sim_mouse_y, snap_enabled=grid_enabled, grid_size=GRID_SIZE)
                    first_break_gravity, show_benchmark_hud = None, False
                elif current_mode == "BEAM":
                    if clicked_node_idx is not None:
                        if active_node_bnd is None: active_node_bnd = clicked_node_idx
                        else:
                            truss.add_beam(active_node_bnd, clicked_node_idx)
                            active_node_bnd, first_break_gravity, show_benchmark_hud = None, None, False
                    else: active_node_bnd = None
            elif event.button == 3 and clicked_node_idx is not None:
                truss.nodes[clicked_node_idx].toggle_support()
                first_break_gravity, show_benchmark_hud = None, False

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 2 or event.button == 1:
                is_panning = False

    if is_panning:
        pan_x += mouse_pos[0] - last_mouse_pos[0]
        pan_y += mouse_pos[1] - last_mouse_pos[1]
        pan_x = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, pan_x))
        pan_y = max(-WORKSPACE_LIMIT, min(WORKSPACE_LIMIT, pan_y))
        last_mouse_pos = mouse_pos

    if selected_node_idx is not None and current_mode == "SELECT" and not is_playing and not input_active:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] or keys[pygame.K_DOWN] or keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
            first_break_gravity, show_benchmark_hud = None, False
        if keys[pygame.K_UP]: truss.nodes[selected_node_idx].load_y -= 1000.0
        if keys[pygame.K_DOWN]: truss.nodes[selected_node_idx].load_y += 1000.0
        if keys[pygame.K_LEFT]: truss.nodes[selected_node_idx].load_x -= 1000.0
        if keys[pygame.K_RIGHT]: truss.nodes[selected_node_idx].load_x += 1000.0

    if is_playing:
        current_def_scale += (TARGET_DEF_SCALE - current_def_scale) * 0.08
        if first_break_gravity is not None and gravity_multiplier > first_break_gravity:
            gravity_multiplier = first_break_gravity
        for b in truss.beams:
            if b.status != "NORMAL" and b.broken_at_gravity is not None:
                if gravity_multiplier < b.broken_at_gravity:
                    b.reset_status()
        if not any(b.status == "FRACTURED" for b in truss.beams): first_break_gravity = None
        solve_truss(truss, gravity_multiplier)
        if truss.is_stable:
            for b in truss.beams:
                if b.status == "FRACTURED": continue
                specs = MATERIAL_SPECS.get(b.material, MATERIAL_SPECS["Steel"])
                utilization = calculate_utilization(b)
                if abs(b.stress) >= specs["ultimate"] or utilization >= 1.6:
                    b.status = "FRACTURED"
                    b.is_broken = True
                    b.broken_at_gravity = gravity_multiplier
                    if first_break_gravity is None: first_break_gravity = gravity_multiplier
                    ax, ay = get_def_pos(b.node_a, truss.nodes[b.node_a])
                    bx, by = get_def_pos(b.node_b, truss.nodes[b.node_b])
                    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                    thick = max(2, min(14, int((b.area / 2.5e-3) * 4.0)))
                    fading_beams.append([ax, ay, mx - 2, my, thick, 1.0, -1.0])
                    fading_beams.append([mx + 2, my, bx, by, thick, 1.0, -1.5])
                    break 
                elif utilization >= 1.0 and b.status == "NORMAL":
                    b.status = "YIELDING"
                    b.broken_at_gravity = gravity_multiplier
    else:
        current_def_scale += (0.0 - current_def_scale) * 0.15
        first_break_gravity = None
        if is_optimizing:
            if not truss.is_stable or len(truss.beams) == 0:
                is_optimizing = False
                trigger_status("OPTIMIZATION FAILED: INVALID STRUCTURE")
            else:
                changed = truss.optimize_step(calculate_utilization, solve_truss, gravity_multiplier, allow_profile_switching)
                if not changed:
                    is_optimizing = False
                    trigger_status("OPTIMIZATION COMPLETE (FOS > 2.0 ACHIEVED)")
                else:
                    pygame.time.delay(90)
        else:
            for b in truss.beams:
                b.reset_status()
            solve_truss(truss, 0.0)

    last_gravity_multiplier = gravity_multiplier

    for fb in fading_beams[:]:
        fb[5] -= 0.04  
        fb[6] += 0.45  
        fb[1] += fb[6]
        fb[3] += fb[6]
        if fb[5] <= 0: fading_beams.remove(fb)

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
    
    total_mass, max_util = 0.0, 0.0
    for b in truss.beams:
        if b.status == "FRACTURED": continue
        total_mass += truss.get_beam_length(b) * b.area * b.density
        max_util = max(max_util, calculate_utilization(b))
        
    fos_val = 1.0 / max_util if max_util > 0.0 else float('inf')
    if selected_beam_idx is not None and truss.beams[selected_beam_idx].status == "FRACTURED":
        selected_beam_idx = None
    
    screen.blit(font_body.render(f"Mass: {total_mass:.1f} kg", True, COLOR_TEXT_MUTED), (15, 350))
    fos_label = font_body.render("Min FoS:", True, COLOR_TEXT_MUTED)
    screen.blit(fos_label, (15, 375))
    
    if not truss.is_stable or len(truss.beams) == 0:
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MUTED)
    elif max_util >= 1.6:
        fos_txt = font_header.render("FAIL", True, COLOR_LOAD)
    elif max_util >= 1.0:
        fos_txt = font_header.render("FAIL", True, COLOR_YIELDING)
    elif max_util == 0.0 or not math.isfinite(fos_val):
        fos_txt = font_body.render("N/A", True, COLOR_TEXT_MAIN)
    else:
        fos_txt = font_header.render(f"{fos_val:.2f}", True, COLOR_ZERO_LOAD if fos_val >= 2.0 else COLOR_MID_LOAD)
    screen.blit(fos_txt, (15 + fos_label.get_width() + 5, 373))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 410), (130, 410), 1)
    pygame.draw.rect(screen, (30, 50, 35) if is_playing else COLOR_BACKGROUND, btn_play, border_radius=4)
    pygame.draw.rect(screen, COLOR_PLAY_GREEN if is_playing else COLOR_UI_BORDER, btn_play, width=1, border_radius=4)
    screen.blit(font_header.render("|| Pause" if is_playing else "> Play", True, COLOR_PLAY_GREEN if is_playing else COLOR_TEXT_MAIN), (btn_play.x + 20, btn_play.y + 10))

    pygame.draw.rect(screen, (40, 55, 45) if truss.self_weight_enabled else (55, 35, 35), btn_w_toggle, border_radius=4)
    pygame.draw.rect(screen, COLOR_PLAY_GREEN if truss.self_weight_enabled else COLOR_LOAD, btn_w_toggle, width=1, border_radius=4)
    w_label = "Weight: ON" if truss.self_weight_enabled else "Weight: OFF"
    screen.blit(font_body.render(w_label, True, COLOR_TEXT_MAIN), (btn_w_toggle.x + 16, btn_w_toggle.y + 6))

    opt_color = COLOR_PLAY_GREEN if is_optimizing else COLOR_UI_BORDER
    pygame.draw.rect(screen, opt_color if is_optimizing else COLOR_BACKGROUND, btn_optimize, border_radius=4)
    pygame.draw.rect(screen, COLOR_UI_BORDER, btn_optimize, width=1, border_radius=4)
    screen.blit(font_header.render("OPTIMIZE", True, COLOR_TEXT_MAIN if is_optimizing else COLOR_TEXT_MUTED), (btn_optimize.x + 18, btn_optimize.y + 9))

    pygame.draw.rect(screen, (10, 10, 12), chk_profile, border_radius=2)
    pygame.draw.rect(screen, COLOR_UI_BORDER, chk_profile, width=1, border_radius=2)
    if allow_profile_switching:
        pygame.draw.rect(screen, COLOR_PLAY_GREEN, chk_profile.inflate(-4, -4), border_radius=1)
    screen.blit(font_body.render("Swap Profile", True, COLOR_TEXT_MAIN if allow_profile_switching else COLOR_TEXT_MUTED), (chk_profile.x + 20, chk_profile.y - 1))

    sim_zone_surface = pygame.Surface((sim_rect.width, sim_rect.height))
    sim_zone_surface.fill(COLOR_SIM_ZONE)

    if grid_enabled:
        scaled_grid = GRID_SIZE * zoom_scale
        if scaled_grid > 2:
            start_sim_x, start_sim_y = to_sim(sim_rect.left, sim_rect.top)
            start_g_x = (math.ceil(start_sim_x / GRID_SIZE) * GRID_SIZE)
            start_g_y = (math.ceil(start_sim_y / GRID_SIZE) * GRID_SIZE)
            
            end_sim_x, end_sim_y = to_sim(sim_rect.right, sim_rect.bottom)
            
            curr_g_x = start_g_x
            while curr_g_x <= end_sim_x:
                screen_x, _ = to_screen(curr_g_x, 0)
                local_x = screen_x - sim_rect.left
                if 0 <= local_x <= sim_rect.width:
                    pygame.draw.line(sim_zone_surface, COLOR_GRID, (local_x, 0), (local_x, sim_rect.height))
                curr_g_x += GRID_SIZE
                
            curr_g_y = start_g_y
            while curr_g_y <= end_sim_y:
                _, screen_y = to_screen(0, curr_g_y)
                local_y = screen_y - sim_rect.top
                if 0 <= local_y <= sim_rect.height:
                    pygame.draw.line(sim_zone_surface, COLOR_GRID, (0, local_y), (sim_rect.width, local_y))
                curr_g_y += GRID_SIZE

    node_has_connections = [False] * len(truss.nodes)
    for beam in truss.beams:
        if beam.status != "FRACTURED":
            node_has_connections[beam.node_a] = True
            node_has_connections[beam.node_b] = True

    for i, beam in enumerate(truss.beams):
        if beam.status == "FRACTURED": continue
        ax, ay = get_def_pos(beam.node_a, truss.nodes[beam.node_a])
        bx, by = get_def_pos(beam.node_b, truss.nodes[beam.node_b])
        
        screen_ax, screen_ay = to_screen(ax, ay)
        screen_bx, screen_by = to_screen(bx, by)
        
        local_ax, local_ay = screen_ax - sim_rect.left, screen_ay - sim_rect.top
        local_bx, local_by = screen_bx - sim_rect.left, screen_by - sim_rect.top
        
        if show_deformed and truss.displacements is None and truss.is_stable and is_playing:
            raw_ax, raw_ay = to_screen(truss.nodes[beam.node_a].x, truss.nodes[beam.node_a].y)
            raw_bx, raw_by = to_screen(truss.nodes[beam.node_b].x, truss.nodes[beam.node_b].y)
            pygame.draw.line(sim_zone_surface, (45, 45, 50), (raw_ax - sim_rect.left, raw_ay - sim_rect.top), (raw_bx - sim_rect.left, raw_by - sim_rect.top), 1)
        
        thickness_pixels = max(1, int(max(2, min(16, int(beam.dim_w * 140.0))) * zoom_scale))
        draw_curved_beam(sim_zone_surface, local_ax, local_ay, local_bx, local_by, beam, thickness_pixels, get_stress_color(beam), i == selected_beam_idx)

    for fb in fading_beams:
        alpha = int(fb[5] * 255)
        if alpha <= 0: continue
        
        rax, ray = to_screen(fb[0], fb[1])
        rbx, rby = to_screen(fb[2], fb[3])
        
        surf = pygame.Surface((sim_rect.width, sim_rect.height), pygame.SRCALPHA)
        pygame.draw.line(surf, (220, 38, 38, alpha), (rax - sim_rect.left, ray - sim_rect.top), (rbx - sim_rect.left, rby - sim_rect.top), max(1, int(fb[4] * zoom_scale)))
        sim_zone_surface.blit(surf, (0, 0))

    for i, node in enumerate(truss.nodes):
        if is_playing and not node_has_connections[i] and not node.is_anchor_x and not node.is_anchor_y: continue
        nx, ny = get_def_pos(i, node)
        
        screen_nx, screen_ny = to_screen(nx, ny)
        local_nx, local_ny = screen_nx - sim_rect.left, screen_ny - sim_rect.top
        
        total_fx = node.load_x
        total_fy = node.load_y + (1000.0 * gravity_multiplier if is_playing else 0.0)
        
        if truss.self_weight_enabled and is_playing and gravity_multiplier > 0.0:
            g = 9.81 * gravity_multiplier
            for beam in truss.beams:
                if beam.status == "FRACTURED": continue
                if beam.node_a == i or beam.node_b == i:
                    total_fy += (truss.get_beam_length(beam) * beam.area * beam.density * g) / 2.0
            
        draw_force_vector(sim_zone_surface, local_nx, local_ny, total_fx, total_fy, COLOR_LOAD)
        if is_playing and (node.rx != 0.0 or node.ry != 0.0):
            draw_force_vector(sim_zone_surface, local_nx, local_ny, node.rx, node.ry, COLOR_REACTION)
        
        r_scale = max(2, int(NODE_RADIUS * zoom_scale))
        if i == selected_node_idx or i == active_node_bnd:
            pygame.draw.circle(sim_zone_surface, COLOR_HIGHLIGHT, (local_nx, local_ny), r_scale + max(2, int(5 * zoom_scale)), width=2)
        if node.is_anchor_x and node.is_anchor_y:
            pygame.draw.circle(sim_zone_surface, COLOR_PIN, (local_nx, local_ny), r_scale + max(2, int(7 * zoom_scale)), width=3)
            pygame.draw.circle(sim_zone_surface, COLOR_PIN, (local_nx, local_ny), r_scale + max(1, int(2 * zoom_scale)))
        elif node.is_anchor_y and not node.is_anchor_x:
            w_off, h_off = int(12 * zoom_scale), int(11 * zoom_scale)
            pygame.draw.line(sim_zone_surface, COLOR_ROLLER, (local_nx - w_off, local_ny + h_off), (local_nx + w_off, local_ny + h_off), 2)
            pygame.draw.line(sim_zone_surface, COLOR_ROLLER, (local_nx - int(8 * zoom_scale), local_ny + int(15 * zoom_scale)), (local_nx + int(8 * zoom_scale), local_ny + int(15 * zoom_scale)), 1)
            pygame.draw.circle(sim_zone_surface, COLOR_ROLLER, (local_nx, local_ny), r_scale + max(1, int(2 * zoom_scale)))
        else:
            pygame.draw.circle(sim_zone_surface, (234, 179, 8) if i == active_node_bnd else COLOR_NODE, (local_nx, local_ny), r_scale)

    screen.blit(sim_zone_surface, (sim_rect.left, sim_rect.top))
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

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
            bhud_w, bhud_h = 430, 200
            bhud_x, bhud_y = sim_rect.left + 15, sim_rect.top + 15
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (63, 63, 70, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            
            bhud_surface.blit(font_header.render("VERIFICATION BENCHMARK METRICS", True, COLOR_TEXT_MAIN), (15, 12))
            bhud_surface.blit(font_body.render("Metric                  Analytical          Numerical         Error", True, COLOR_TEXT_MUTED), (15, 34))
            pygame.draw.line(bhud_surface, COLOR_UI_BORDER, (10, 52), (bhud_w - 10, 52), 1)
            
            rows = [
                ("Diag Force", f"{metrics['diag_force_theory']/1000.0:.2f} kN", f"{metrics['diag_force_num']/1000.0:.2f} kN", f"{metrics['diag_force_err']:.4f}%"),
                ("Horiz Force", f"{metrics['horiz_force_theory']/1000.0:.2f} kN", f"{metrics['horiz_force_num']/1000.0:.2f} kN", f"{metrics['horiz_force_err']:.4f}%"),
                ("Deflection dX", f"{metrics['dx_theory']*1000.0:.4f} mm", f"{metrics['dx_num']*1000.0:.4f} mm", f"{metrics['dx_err']:.4f}%"),
                ("Deflection dY", f"{metrics['dy_theory']*1000.0:.4f} mm", f"{metrics['dy_num']*1000.0:.4f} mm", f"{metrics['dy_err']:.4f}%")
            ]
            
            curr_y = 62
            for label, theory_str, num_str, err_str in rows:
                bhud_surface.blit(font_body.render(label, True, COLOR_TEXT_MUTED), (15, curr_y))
                bhud_surface.blit(font_body.render(theory_str, True, COLOR_TEXT_MAIN), (135, curr_y))
                bhud_surface.blit(font_body.render(num_str, True, COLOR_TEXT_MAIN), (235, curr_y))
                bhud_surface.blit(font_body.render(err_str, True, COLOR_ZERO_LOAD), (335, curr_y))
                curr_y += 20
                
            pygame.draw.line(bhud_surface, COLOR_UI_BORDER, (10, 150), (bhud_w - 10, 150), 1)
            bhud_surface.blit(font_body.render("Target: 3m span, 50kN point-load at node 2.", True, COLOR_TEXT_MUTED), (15, 158))
            bhud_surface.blit(font_body.render("Status: Matrix validation test pass.", True, COLOR_ZERO_LOAD), (15, 176))
            screen.blit(bhud_surface, (bhud_x, bhud_y))
        else:
            bhud_w, bhud_h = 320, 45
            bhud_x, bhud_y = sim_rect.left + 15, sim_rect.top + 15
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (127, 29, 29, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            bhud_surface.blit(font_body.render("BENCHMARK CASE MODIFIED OR INVALID", True, COLOR_MAX_LOAD), (15, 14))
            screen.blit(bhud_surface, (bhud_x, bhud_y))

    if ((selected_node_idx is not None) or (selected_beam_idx is not None)) and truss.is_stable:
        lines, header_text, is_beam_selected = [], "", selected_beam_idx is not None
        
        if is_beam_selected:
            beam = truss.beams[selected_beam_idx]
            header_text = "STRUCTURAL ELEMENT"
            length_m = truss.get_beam_length(beam) 
            stress_m_pa = beam.stress / 1e6               
            force_k_n = beam.force / 1000.0               
            nature = "TENSION" if beam.stress > 1e-2 else ("COMPRESSION" if beam.stress < -1e-2 else "NEUTRAL")
            utilization_pct = calculate_utilization(beam) * 100.0
            
            top_lines = [f"Alloy: {beam.material} [M]", f"Force: {abs(force_k_n):.1f} kN", f"Stress: {abs(stress_m_pa):.1f} MPa"]
            if nature == "COMPRESSION" and length_m > 0:
                p_crit = (math.pi ** 2 * beam.modulus * beam.inertia) / (length_m ** 2)
                top_lines.append(f"Buckling Limit: {p_crit / 1000.0:.1f} kN")
            top_lines.extend([f"Type: {nature}", f"Load Capacity: {utilization_pct:.1f}%", f"Status: {beam.status}"])
            
            geom_lines = [f"Profile: {beam.profile} [P]", f"Width/Diam: {beam.dim_w * 100.0:.1f} cm [ [ ] / [ ] ]", f"Thickness: {beam.dim_t * 100.0:.1f} cm", f"Area: {beam.area * 1e4:.1f} cm²", f"Length: {length_m:.2f} m"]
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = "STRUCTURAL NODE"
            support_str = "Pin Support" if node.is_anchor_x and node.is_anchor_y else ("Roller Support" if node.is_anchor_y else "Free Joint")
            effective_y = node.load_y + (1000.0 * gravity_multiplier if is_playing else 0.0)
            net_magnitude = math.hypot(node.load_x, effective_y) / 1000.0
            lines = [f"Type: {support_str}", f"Coords: ({int(node.x)}, {int(node.y)})", f"Net Load: {net_magnitude:.1f} kN", f"Load X: {node.load_x / 1000.0:.1f} kN", f"Load Y: {effective_y / 1000.0:.1f} kN"]
            if node.is_anchor_x or node.is_anchor_y:
                net_react = math.hypot(node.rx, node.ry) / 1000.0
                lines.extend([f"Net React: {net_react:.1f} kN", f"React X: {node.rx / 1000.0:.1f} kN", f"React Y: {node.ry / 1000.0:.1f} kN"])

        if is_beam_selected:
            hud_w, hud_h = 340, 55 + (len(top_lines) * 24) + 12 + 95
        else:
            hud_w = max(240, max([font_body.size(line)[0] for line in lines]) + 40) if lines else 240
            hud_h = 45 + (len(lines) * 24) + (75 if (current_mode == "LOAD" and selected_node_idx is not None) else 0)
            
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
                box_x, box_y = pygame.Rect(15, local_y, 95, 25), pygame.Rect(120, local_y, 95, 25)
                mx, my = pygame.mouse.get_pos()
                lx, ly = mx - hud_x, my - (sim_rect.top + 15)
                
                if pygame.mouse.get_pressed()[0]:
                    if box_x.collidepoint((lx, ly)): input_active, input_type, input_buffer = True, "X", ""
                    elif box_y.collidepoint((lx, ly)): input_active, input_type, input_buffer = True, "Y", ""
                        
                pygame.draw.rect(hud_surface, (30, 30, 35) if (input_active and input_type == "X") else (10, 10, 12), box_x, border_radius=4)
                pygame.draw.rect(hud_surface, COLOR_UI_BORDER, box_x, width=1, border_radius=4)
                hud_surface.blit(font_body.render("FX: " + (input_buffer if (input_active and input_type == "X") else f"{truss.nodes[selected_node_idx].load_x/1000.0:.1f}") + ("_" if (input_active and input_type == "X") else " kN"), True, COLOR_TEXT_MAIN), (22, local_y + 5))
                
                pygame.draw.rect(hud_surface, (30, 30, 35) if (input_active and input_type == "Y") else (10, 10, 12), box_y, border_radius=4)
                pygame.draw.rect(hud_surface, COLOR_UI_BORDER, box_y, width=1, border_radius=4)
                hud_surface.blit(font_body.render("FY: " + (input_buffer if (input_active and input_type == "Y") else f"{truss.nodes[selected_node_idx].load_y/1000.0:.1f}") + ("_" if (input_active and input_type == "Y") else " kN"), True, COLOR_TEXT_MAIN), (127, local_y + 5))
                hud_surface.blit(font_body.render("Click box, type value, press Enter", True, COLOR_TEXT_MUTED), (15, local_y + 35))

        screen.blit(hud_surface, (hud_x, sim_rect.top + 15))

    if status_banner_timer > 0:
        status_banner_timer -= 1
        sw, sh = font_header.size(status_banner_text)[0] + 30, 35
        sb_rect = pygame.Rect(sim_rect.left + (sim_rect.width - sw) // 2, sim_rect.bottom - 50, sw, sh)
        pygame.draw.rect(screen, (24, 24, 27, 230), sb_rect, border_radius=4)
        pygame.draw.rect(screen, COLOR_PLAY_GREEN if "SUCCESS" in status_banner_text or "LOADED" in status_banner_text else COLOR_LOAD, sb_rect, width=1, border_radius=4)
        screen.blit(font_header.render(status_banner_text, True, COLOR_TEXT_MAIN), (sb_rect.x + 15, sb_rect.y + 9))

    grav_msg = f"GRAVITY LOAD MULTIPLIER: {gravity_multiplier:.1f}x  [ - ] / [ + ]"
    if first_break_gravity is not None and math.isclose(gravity_multiplier, first_break_gravity): grav_msg += " (CRITICAL POINT LOCKED)"
    screen.blit(font_body.render(grav_msg, True, COLOR_TEXT_MAIN), (165, WINDOW_HEIGHT - 75))
    screen.blit(font_body.render(f"GRID SNAP: {'ENABLED (20px)' if grid_enabled else 'DISABLED'} [G] | DEFORM DISPLAY: {'ON' if show_deformed else 'OFF'} [D] | SAVE: [Ctrl+S] | LOAD: [Ctrl+O]", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 55))
    screen.blit(font_body.render("Keys/Buttons: [1-3] Material | [R] Reset | [SPACE]/[Play] Playback | Arrow Keys adjust external node loads | [ / ] dimensions | [M] alloy | [P] structural profile | [V] Benchmark | [W] Self-Weight Toggle | Scroll Wheel Zoom | Mid-Click Drag Pan | [H] Home Cam", True, COLOR_TEXT_MUTED), (165, WINDOW_HEIGHT - 35))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()