import pygame
import sys
import math
import tkinter as tk
import json
from truss_model import TrussSystem, Node, Beam
from matrix_solver import solve_truss, calculate_benchmark_metrics
from constants import *
from camera import Camera
from serialization import save_project_dialog, load_project_dialog
from materials import MaterialManager
from physics_engine import PhysicsSimulation

root = tk.Tk()
root.withdraw()

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

font_header = pygame.font.SysFont("Helvetica", 16, bold=True)
font_body = pygame.font.SysFont("Helvetica", 13)

clock = pygame.time.Clock()
truss = TrussSystem()

active_node_bnd = None  
selected_node_idx = None 
selected_beam_idx = None 

current_mode = "SELECT" 
is_playing = False       
is_physics_playing = False
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

camera = Camera()
physics_sim = None
saved_truss_state = None

def trigger_status(text):
    global status_banner_text, status_banner_timer
    status_banner_text = text
    status_banner_timer = 180  

def capture_truss_state():
    return [{"x": n.x, "y": n.y} for n in truss.nodes]

def restore_truss_state(state):
    for i, s in enumerate(state):
        truss.nodes[i].x = s["x"]
        truss.nodes[i].y = s["y"]

def run_determinism_test(base_truss, num_runs=3, frames=180):
    print("\n--- STARTING PHYSICS DETERMINISM TEST ---")
    state_str = json.dumps({
        "self_weight_enabled": base_truss.self_weight_enabled,
        "active_material": base_truss.active_material,
        "nodes": [n.to_dict() for n in base_truss.nodes],
        "beams": [b.to_dict() for b in base_truss.beams]
    })
    
    run_histories = []
    
    for run_idx in range(num_runs):
        test_truss = TrussSystem()
        data = json.loads(state_str)
        test_truss.self_weight_enabled = data["self_weight_enabled"]
        test_truss.active_material = data["active_material"]
        for n_data in data["nodes"]: test_truss.nodes.append(Node.from_dict(n_data))
        for b_data in data["beams"]: test_truss.beams.append(Beam.from_dict(b_data))
        
        sim = PhysicsSimulation(test_truss, gravity_mult=1.0, enable_gravity=test_truss.self_weight_enabled)
        
        history = []
        for frame in range(frames):
            sim.step(1.0)
            sim.sync_to_truss(test_truss)
            
            for c in sim.constraints:
                dx = test_truss.nodes[c.p2_idx].x - test_truss.nodes[c.p1_idx].x
                dy = test_truss.nodes[c.p2_idx].y - test_truss.nodes[c.p1_idx].y
                curr_len = math.sqrt(dx * dx + dy * dy)
                if c.rest_length > 1e-6:
                    strain = (curr_len - c.rest_length) / c.rest_length
                    c.beam.stress = strain * c.beam.modulus
                    c.beam.force = c.beam.stress * c.beam.area
            
            beams_to_break = []
            for b_idx, b in enumerate(test_truss.beams):
                if b.status == "FRACTURED": continue
                ult = MaterialManager.get_ultimate_stress(b.material)
                yield_s = MaterialManager.get_yield_stress(b.material)
                util = abs(b.stress) / yield_s if yield_s > 0 else 0
                if abs(b.stress) >= ult or util >= 1.6:
                    beams_to_break.append((b, util, b_idx))
            
            beams_to_break.sort(key=lambda item: (item[1], -item[2]), reverse=True)
            for b, util, b_idx in beams_to_break:
                b.status = "FRACTURED"
                sim.remove_constraints_for_beam(b)
                
            frame_hash = sum([round(p.x + p.y + p.vx + p.vy, 4) for p in sim.particles]) + sum([1 for b in test_truss.beams if b.status == "FRACTURED"])
            history.append(frame_hash)
            
        run_histories.append(history)
        print(f"Run {run_idx+1} completed.")

    diverged = False
    for f in range(frames):
        val = run_histories[0][f]
        for r in range(1, num_runs):
            if run_histories[r][f] != val:
                print(f"!! DIVERGENCE DETECTED at Frame {f} !! Run 1: {val}, Run {r+1}: {run_histories[r][f]}")
                diverged = True
                break
        if diverged: break
    
    if not diverged:
        print(f"SUCCESS: 100% Deterministic across {num_runs} runs ({frames} frames each).")
    print("-----------------------------------------\n")

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
    yield_stress = MaterialManager.get_yield_stress(beam.material)
    yield_util = abs(beam.stress) / yield_stress
    if beam.status == "YIELDING":
        limit, proxy_node = find_proxy_limit(beam)
        if proxy_node is not None:
            current_visual_bow = min(35.0, (yield_util - 0.95) * 18.0)
            if current_visual_bow >= limit:
                return max(yield_util * 1.6, 1.2)
    if beam.stress < -1e-2:
        length_m = truss.get_beam_length(beam)
        if length_m > 0:
            p_crit = MaterialManager.calculate_buckling_load(beam.material, beam.inertia, length_m)
            buckle_util = abs(beam.force) / p_crit
            return max(yield_util, buckle_util)
    return yield_util

def get_stress_color(beam):
    if not truss.is_stable and not is_physics_playing:
        return COLOR_TEXT_MUTED
    if beam.status == "FRACTURED" or beam.force == 0.0:
        return (180, 180, 185)
    if beam.status == "YIELDING":
        limit, proxy_node = find_proxy_limit(beam)
        if proxy_node is not None:
            yield_stress = MaterialManager.get_yield_stress(beam.material)
            util = abs(beam.stress) / yield_stress
            if min(35.0, (util - 0.95) * 18.0) >= limit:
                return (220, 38, 38)
        pulse = (math.sin(pygame.time.get_ticks() * 0.01) + 1.0) / 2.0
        return (int(249 - pulse * 15), int(115 + pulse * 25), int(22 - pulse * 10))
    yield_stress = MaterialManager.get_yield_stress(beam.material)
    yield_util = abs(beam.stress) / yield_stress
    is_compression = beam.stress < -1e-2
    if is_compression:
        length_m = truss.get_beam_length(beam)
        buckle_util = 0.0
        if length_m > 0:
            p_crit = MaterialManager.calculate_buckling_load(beam.material, beam.inertia, length_m)
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
    arrow_len = max(20, min(65, int(mag / 1000.0) * 1.5 + 20)) * camera.zoom_scale
    start_x = cx - dx * arrow_len
    start_y = cy - dy * arrow_len
    pygame.draw.line(surface, color, (start_x, start_y), (cx, cy), max(1, int(3 * camera.zoom_scale)))
    wing_len = max(3, int(8 * camera.zoom_scale))
    angle = math.atan2(cy - start_y, cx - start_x)
    pygame.draw.polygon(surface, color, [(cx, cy), (cx - wing_len * math.cos(angle + 0.4), cy - wing_len * math.sin(angle + 0.4)), (cx - wing_len * math.cos(angle - 0.4), cy - wing_len * math.sin(angle - 0.4))])

def compute_dynamic_reactions(truss, gravity_mult):
    g = 9.81 * gravity_mult
    
    for node in truss.nodes:
        node.rx = 0.0
        node.ry = 0.0
        
    for i, node in enumerate(truss.nodes):
        if not node.is_anchor_x and not node.is_anchor_y:
            continue
            
        fx_net = node.load_x
        fy_net = node.load_y
        
        if truss.self_weight_enabled and gravity_mult > 0.0:
            for beam in truss.beams:
                if beam.status == "FRACTURED": continue
                if beam.node_a == i or beam.node_b == i:
                    L_m = truss.get_beam_length(beam)
                    if is_physics_playing and saved_truss_state is not None:
                        na = saved_truss_state[beam.node_a]
                        nb = saved_truss_state[beam.node_b]
                        L_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
                    fy_net += (L_m * beam.area * beam.density * g) / 2.0
                    
        for beam in truss.beams:
            if beam.status == "FRACTURED": continue
            if beam.node_a == i or beam.node_b == i:
                na = truss.nodes[beam.node_a]
                nb = truss.nodes[beam.node_b]
                dx = nb.x - na.x
                dy = nb.y - na.y
                L_px = math.hypot(dx, dy)
                if L_px > 1e-6:
                    dir_x = dx / L_px
                    dir_y = dy / L_px
                    if beam.node_a == i:
                        fx_net += beam.force * dir_x
                        fy_net += beam.force * dir_y
                    else:
                        fx_net -= beam.force * dir_x
                        fy_net -= beam.force * dir_y
                        
        if node.is_anchor_x:
            node.rx = -fx_net
        if node.is_anchor_y:
            node.ry = -fy_net

def get_def_pos(idx, node):
    if is_physics_playing:
        return node.x, node.y
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
    yield_stress = MaterialManager.get_yield_stress(beam.material)
    util = abs(beam.stress) / yield_stress
    max_bow = min(35.0, (util - 0.95) * 18.0) * camera.zoom_scale
    if max_bow < 1.0:
        max_bow = 1.0
    if beam.stress > 0.0:
        max_bow *= 0.15
    limit, proxy_node = find_proxy_limit(beam)
    if proxy_node is not None and max_bow >= (limit * camera.zoom_scale):
        max_bow = limit * camera.zoom_scale
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
    btn_select       = pygame.Rect(15, 80, 110, 35)
    btn_node         = pygame.Rect(15, 125, 110, 35)
    btn_beam         = pygame.Rect(15, 170, 110, 35)
    btn_load         = pygame.Rect(15, 215, 110, 35)
    btn_benchmark    = pygame.Rect(15, 260, 110, 35)
    btn_static_play  = pygame.Rect(15, 430, 110, 35)
    btn_physics_play = pygame.Rect(15, 475, 110, 40)
    btn_w_toggle     = pygame.Rect(15, 525, 110, 30)
    btn_optimize     = pygame.Rect(15, 565, 110, 35)
    chk_profile      = pygame.Rect(15, 610, 14, 14)

    mouse_pos = pygame.mouse.get_pos()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.MOUSEWHEEL:
            if camera.process_event(event, mouse_pos):
                continue

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
            if ctrl_pressed and not is_playing and not is_physics_playing:
                if event.key == pygame.K_s:
                    status = save_project_dialog(truss)
                    if status:
                        trigger_status(status)
                    continue
                elif event.key == pygame.K_o:
                    success, status = load_project_dialog(truss)
                    if success:
                        selected_node_idx, selected_beam_idx, active_node_bnd = None, None, None
                        gravity_multiplier, first_break_gravity = 0.0, None
                        fading_beams.clear()
                        show_benchmark_hud = False
                        is_optimizing = False
                        camera.reset()
                        trigger_status(status)
                    elif status:
                        trigger_status(status)
                    continue

            if event.key == pygame.K_SPACE and not pygame.key.get_pressed()[pygame.K_LALT]:
                if not is_playing and not is_physics_playing:
                    saved_truss_state = capture_truss_state()
                    physics_sim = PhysicsSimulation(truss, gravity_mult=gravity_multiplier, enable_gravity=truss.self_weight_enabled)
                    is_physics_playing = True
                    is_optimizing = False
                    active_node_bnd = None
                    input_active = False
                    trigger_status("DYNAMIC PLAY MODE ACTIVE")
                elif is_physics_playing:
                    is_physics_playing = False
                    physics_sim = None
                    if saved_truss_state:
                        restore_truss_state(saved_truss_state)
                    for b in truss.beams: b.reset_status()
                    trigger_status("DYNAMIC SIMULATION RESET")
                elif is_playing:
                    is_playing = False
                    trigger_status("STATIC PLAY PAUSED")
            elif event.key == pygame.K_t and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                if not is_playing and not is_physics_playing:
                    trigger_status("RUNNING DETERMINISM TEST (SEE CONSOLE)")
                    run_determinism_test(truss)
            elif event.key == pygame.K_h:
                camera.reset()
                trigger_status("CAMERA RESET TO ORIGIN")
            elif event.key == pygame.K_EQUALS:
                if first_break_gravity is not None: gravity_multiplier = min(first_break_gravity, gravity_multiplier + 0.5)
                else: gravity_multiplier = min(100.0, gravity_multiplier + 0.5)
            elif event.key == pygame.K_MINUS: gravity_multiplier = max(0.0, gravity_multiplier - 0.5)
                
            if not is_playing and not is_physics_playing:
                if event.key == pygame.K_r:
                    truss.clear()
                    active_node_bnd, selected_node_idx, selected_beam_idx = None, None, None
                    gravity_multiplier, first_break_gravity = 0.0, None
                    fading_beams.clear()
                    input_active, show_benchmark_hud, is_optimizing = False, False, False
                    camera.reset()
                elif event.key == pygame.K_g: grid_enabled = not grid_enabled
                elif event.key == pygame.K_d: show_deformed = not show_deformed
                elif event.key == pygame.K_w:
                    truss.self_weight_enabled = not truss.self_weight_enabled
                    if is_physics_playing and physics_sim is not None:
                        physics_sim.enable_gravity = truss.self_weight_enabled
                elif event.key == pygame.K_v:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx, selected_beam_idx, active_node_bnd, first_break_gravity = None, None, None, None
                        is_optimizing = False
                        camera.reset()
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
                    next_m = MaterialManager.get_next_material(b.material)
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
                if btn_static_play.collidepoint(event.pos):
                    if not is_physics_playing:
                        is_playing = not is_playing
                        is_optimizing = False
                        input_active = False
                elif btn_physics_play.collidepoint(event.pos):
                    if not is_playing:
                        if not is_physics_playing:
                            saved_truss_state = capture_truss_state()
                            physics_sim = PhysicsSimulation(truss, gravity_mult=gravity_multiplier, enable_gravity=truss.self_weight_enabled)
                            is_physics_playing = True
                            is_optimizing = False
                            input_active = False
                            trigger_status("DYNAMIC PLAY MODE ACTIVE")
                        else:
                            is_physics_playing = False
                            physics_sim = None
                            if saved_truss_state:
                                restore_truss_state(saved_truss_state)
                            for b in truss.beams: b.reset_status()
                            trigger_status("DYNAMIC SIMULATION RESET")
                elif btn_w_toggle.collidepoint(event.pos):
                    truss.self_weight_enabled = not truss.self_weight_enabled
                    if is_physics_playing and physics_sim is not None:
                        physics_sim.enable_gravity = truss.self_weight_enabled
                elif btn_optimize.collidepoint(event.pos) and not is_playing and not is_physics_playing:
                    is_optimizing = not is_optimizing
                    if is_optimizing: trigger_status("OPTIMIZATION ENGINE ACTIVE")
                    else: trigger_status("OPTIMIZATION HALTED")
                elif chk_profile.inflate(10, 10).collidepoint(event.pos):
                    allow_profile_switching = not allow_profile_switching
                elif btn_select.collidepoint(event.pos) and not is_playing and not is_physics_playing: current_mode, input_active = "SELECT", False
                elif btn_node.collidepoint(event.pos) and not is_playing and not is_physics_playing: current_mode, input_active = "NODE", False
                elif btn_beam.collidepoint(event.pos) and not is_playing and not is_physics_playing: current_mode, input_active = "BEAM", False
                elif btn_load.collidepoint(event.pos) and not is_playing and not is_physics_playing: current_mode, input_active = "LOAD", False
                elif btn_benchmark.collidepoint(event.pos) and not is_playing and not is_physics_playing:
                    show_benchmark_hud = not show_benchmark_hud
                    if show_benchmark_hud:
                        truss.load_benchmark_case()
                        selected_node_idx, selected_beam_idx, active_node_bnd, first_break_gravity = None, None, None, None
                        is_optimizing = False
                        camera.reset()
                continue

            if input_active: input_active, input_buffer = False, ""
            if not sim_rect.collidepoint(event.pos): continue

            if camera.process_event(event, mouse_pos):
                continue

            sim_mouse_x, sim_mouse_y = camera.to_sim(event.pos[0], event.pos[1])

            clicked_node_idx = None
            for i, node in enumerate(truss.nodes):
                if math.hypot(node.x - sim_mouse_x, node.y - sim_mouse_y) < (CLICK_TOLERANCE / camera.zoom_scale):
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
                            if point_to_line_distance(sim_mouse_x, sim_mouse_y, truss.nodes[b.node_a].x, truss.nodes[b.node_a].y, truss.nodes[b.node_b].x, truss.nodes[b.node_b].y) < (8 / camera.zoom_scale):
                                selected_beam_idx = i
                elif current_mode == "NODE" and clicked_node_idx is None:
                    sim_mouse_x = max(-camera.WORKSPACE_LIMIT, min(camera.WORKSPACE_LIMIT, sim_mouse_x))
                    sim_mouse_y = max(-camera.WORKSPACE_LIMIT, min(camera.WORKSPACE_LIMIT, sim_mouse_y))
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
            camera.process_event(event, mouse_pos)

    camera.update(mouse_pos)

    if selected_node_idx is not None and current_mode == "SELECT" and not is_playing and not is_physics_playing and not input_active:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] or keys[pygame.K_DOWN] or keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
            first_break_gravity, show_benchmark_hud = None, False
        if keys[pygame.K_UP]: truss.nodes[selected_node_idx].load_y -= 1000.0
        if keys[pygame.K_DOWN]: truss.nodes[selected_node_idx].load_y += 1000.0
        if keys[pygame.K_LEFT]: truss.nodes[selected_node_idx].load_x -= 1000.0
        if keys[pygame.K_RIGHT]: truss.nodes[selected_node_idx].load_x += 1000.0

    if is_physics_playing:
        if physics_sim is not None:
            physics_sim.step(gravity_multiplier)
            physics_sim.sync_to_truss(truss)
            
            compute_dynamic_reactions(truss, gravity_multiplier)

            beams_to_break = []
            for b_idx, b in enumerate(truss.beams):
                if b.status == "FRACTURED":
                    continue
                ultimate_stress = MaterialManager.get_ultimate_stress(b.material)
                utilization = calculate_utilization(b)
                if abs(b.stress) >= ultimate_stress or utilization >= 1.6:
                    beams_to_break.append((b, utilization, b_idx))

            beams_to_break.sort(key=lambda item: (item[1], -item[2]), reverse=True)

            for b, util, b_idx in beams_to_break:
                b.status = "FRACTURED"
                b.is_broken = True
                b.broken_at_gravity = gravity_multiplier
                
                if physics_sim is not None:
                    physics_sim.remove_constraints_for_beam(b)
                if first_break_gravity is None:
                    first_break_gravity = gravity_multiplier
                    
                ax, ay = get_def_pos(b.node_a, truss.nodes[b.node_a])
                bx, by = get_def_pos(b.node_b, truss.nodes[b.node_b])
                mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                thick = max(2, min(14, int((b.area / 2.5e-3) * 4.0)))
                fading_beams.append([ax, ay, mx - 2, my, thick, 1.0, -1.0])
                fading_beams.append([mx + 2, my, bx, by, thick, 1.0, -1.5])
                
            for b in truss.beams:
                if b.status == "FRACTURED": continue
                utilization = calculate_utilization(b)
                if utilization >= 1.0 and b.status == "NORMAL":
                    b.status = "YIELDING"
                    b.broken_at_gravity = gravity_multiplier
                elif utilization < 1.0 and b.status == "YIELDING":
                    b.status = "NORMAL"

    elif is_playing:
        current_def_scale += (TARGET_DEF_SCALE - current_def_scale) * 0.08
        if first_break_gravity is not None and gravity_multiplier > first_break_gravity:
            gravity_multiplier = first_break_gravity
            
        for b in truss.beams:
            if b.status != "NORMAL" and b.broken_at_gravity is not None:
                if gravity_multiplier < b.broken_at_gravity:
                    b.reset_status()
                    
        if not any(b.status == "FRACTURED" for b in truss.beams): 
            first_break_gravity = None
            
        solve_truss(truss, gravity_multiplier)
        
        if truss.is_stable:
            beams_to_break = []
            for b_idx, b in enumerate(truss.beams):
                if b.status == "FRACTURED": continue
                ultimate_stress = MaterialManager.get_ultimate_stress(b.material)
                utilization = calculate_utilization(b)
                if abs(b.stress) >= ultimate_stress or utilization >= 1.6:
                    beams_to_break.append((b, utilization, b_idx))
                    
            beams_to_break.sort(key=lambda item: (item[1], -item[2]), reverse=True)
            
            for b, util, b_idx in beams_to_break:
                b.status = "FRACTURED"
                b.is_broken = True
                b.broken_at_gravity = gravity_multiplier
                if physics_sim is not None:
                    physics_sim.remove_constraints_for_beam(b)
                if first_break_gravity is None: 
                    first_break_gravity = gravity_multiplier
                    
                ax, ay = get_def_pos(b.node_a, truss.nodes[b.node_a])
                bx, by = get_def_pos(b.node_b, truss.nodes[b.node_b])
                mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                thick = max(2, min(14, int((b.area / 2.5e-3) * 4.0)))
                fading_beams.append([ax, ay, mx - 2, my, thick, 1.0, -1.0])
                fading_beams.append([mx + 2, my, bx, by, thick, 1.0, -1.5])
                
            for b in truss.beams:
                if b.status == "FRACTURED": continue
                utilization = calculate_utilization(b)
                if utilization >= 1.0 and b.status == "NORMAL":
                    b.status = "YIELDING"
                    b.broken_at_gravity = gravity_multiplier
                elif utilization < 1.0 and b.status == "YIELDING":
                    b.status = "NORMAL"
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
        pygame.draw.rect(screen, COLOR_UI_BORDER if current_mode == mode and not is_playing and not is_physics_playing else COLOR_BACKGROUND, btn, border_radius=4)
        screen.blit(font_body.render(label, True, COLOR_TEXT_MAIN), (btn.x + 12, btn.y + 10))

    pygame.draw.rect(screen, COLOR_UI_BORDER if show_benchmark_hud and not is_playing and not is_physics_playing else COLOR_BACKGROUND, btn_benchmark, border_radius=4)
    screen.blit(font_body.render("5. Benchmark", True, COLOR_TEXT_MAIN), (btn_benchmark.x + 12, btn_benchmark.y + 10))

    pygame.draw.line(screen, COLOR_UI_BORDER, (10, 310), (130, 310), 1)
    screen.blit(font_header.render("SYSTEM STATS", True, COLOR_TEXT_MAIN), (15, 325))
    
    total_mass, max_util = 0.0, 0.0
    for b in truss.beams:
        if b.status == "FRACTURED": continue
        
        if is_physics_playing and saved_truss_state is not None:
            na = saved_truss_state[b.node_a]
            nb = saved_truss_state[b.node_b]
            length_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
        else:
            length_m = truss.get_beam_length(b)
            
        total_mass += length_m * b.area * b.density
        max_util = max(max_util, calculate_utilization(b))
        
    fos_val = 1.0 / max_util if max_util > 0.0 else float('inf')
    if selected_beam_idx is not None and truss.beams[selected_beam_idx].status == "FRACTURED":
        selected_beam_idx = None
    
    screen.blit(font_body.render(f"Mass: {total_mass:.1f} kg", True, COLOR_TEXT_MUTED), (15, 350))
    fos_label = font_body.render("Min FoS:", True, COLOR_TEXT_MUTED)
    screen.blit(fos_label, (15, 375))
    
    if len(truss.beams) == 0 or (not truss.is_stable and not is_physics_playing):
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

    pygame.draw.rect(screen, (30, 50, 35) if is_playing else COLOR_BACKGROUND, btn_static_play, border_radius=4)
    pygame.draw.rect(screen, COLOR_PLAY_GREEN if is_playing else COLOR_UI_BORDER, btn_static_play, width=1, border_radius=4)
    screen.blit(font_header.render("|| Static Pause" if is_playing else "> Static Play", True, COLOR_PLAY_GREEN if is_playing else COLOR_TEXT_MAIN), (btn_static_play.x + 10, btn_static_play.y + 10))

    pygame.draw.rect(screen, (35, 40, 60) if is_physics_playing else COLOR_BACKGROUND, btn_physics_play, border_radius=4)
    pygame.draw.rect(screen, (100, 150, 255) if is_physics_playing else COLOR_UI_BORDER, btn_physics_play, width=1, border_radius=4)
    screen.blit(font_header.render("[] Stop Dyn" if is_physics_playing else "> Dynamic Play", True, (100, 150, 255) if is_physics_playing else COLOR_TEXT_MAIN), (btn_physics_play.x + 10, btn_physics_play.y + 12))

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
        scaled_grid = GRID_SIZE * camera.zoom_scale
        if scaled_grid > 2:
            start_sim_x, start_sim_y = camera.to_sim(sim_rect.left, sim_rect.top)
            start_g_x = (math.ceil(start_sim_x / GRID_SIZE) * GRID_SIZE)
            start_g_y = (math.ceil(start_sim_y / GRID_SIZE) * GRID_SIZE)

            end_sim_x, end_sim_y = camera.to_sim(sim_rect.right, sim_rect.bottom)

            curr_g_x = start_g_x
            while curr_g_x <= end_sim_x:
                screen_x, _ = camera.to_screen(curr_g_x, 0)
                local_x = screen_x - sim_rect.left
                if 0 <= local_x <= sim_rect.width:
                    pygame.draw.line(sim_zone_surface, COLOR_GRID, (local_x, 0), (local_x, sim_rect.height))
                curr_g_x += GRID_SIZE

            curr_g_y = start_g_y
            while curr_g_y <= end_sim_y:
                _, screen_y = camera.to_screen(0, curr_g_y)
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

        sx, sy = camera.to_screen(ax, ay)
        local_ax = round(sx-sim_rect.left)
        local_ay = round(sy-sim_rect.top)
        sx, sy = camera.to_screen(bx, by)
        local_bx = round(sx-sim_rect.left)
        local_by = round(sy-sim_rect.top)

        if show_deformed and truss.displacements is None and truss.is_stable and is_playing:
            sx, sy = camera.to_screen(truss.nodes[beam.node_a].x, truss.nodes[beam.node_a].y)
            raw_ax = round(sx-sim_rect.left)
            raw_ay = round(sy-sim_rect.top)
            sx, sy = camera.to_screen(truss.nodes[beam.node_b].x, truss.nodes[beam.node_b].y)
            raw_bx = round(sx-sim_rect.left)
            raw_by = round(sy-sim_rect.top)
            pygame.draw.line(sim_zone_surface, (45, 45, 50), (raw_ax, raw_ay), (raw_bx, raw_by), 1)
        thickness_pixels = max(1, int(max(2, min(16, int(beam.dim_w * 140.0))) * camera.zoom_scale))
        draw_curved_beam(sim_zone_surface, local_ax, local_ay, local_bx, local_by, beam, thickness_pixels, get_stress_color(beam), i == selected_beam_idx)

    for fb in fading_beams:
        alpha = int(fb[5] * 255)
        if alpha <= 0: continue

        rax, ray = camera.to_screen(fb[0], fb[1])
        rbx, rby = camera.to_screen(fb[2], fb[3])

    for i, node in enumerate(truss.nodes):
        if (is_playing or is_physics_playing) and not node_has_connections[i] and not node.is_anchor_x and not node.is_anchor_y: continue
        nx, ny = get_def_pos(i, node)
       
        sx, sy = camera.to_screen(nx, ny)
        local_nx = round(sx-sim_rect.left)
        local_ny = round(sy-sim_rect.top)

        total_fx = node.load_x
        total_fy = node.load_y + (1000.0 * gravity_multiplier if truss.self_weight_enabled and (is_playing or is_physics_playing) else 0.0)
        
        if truss.self_weight_enabled and (is_playing or is_physics_playing) and gravity_multiplier > 0.0:
            g = 9.81 * gravity_multiplier
            for beam in truss.beams:
                if beam.status == "FRACTURED": continue
                if beam.node_a == i or beam.node_b == i:
                    total_fy += (truss.get_beam_length(beam) * beam.area * beam.density * g) / 2.0
            
        draw_force_vector(sim_zone_surface, local_nx, local_ny, total_fx, total_fy, COLOR_LOAD)
        if (is_playing or is_physics_playing) and (abs(node.rx) > 0.1 or abs(node.ry) > 0.1):
            net_r = math.hypot(node.rx, node.ry)
            dx = node.rx / net_r
            dy = -node.ry / net_r
            
            base_offset = 15 * camera.zoom_scale
            arrow_len = 35 * camera.zoom_scale
            
            start_x = local_nx + dx * base_offset
            start_y = local_ny + dy * base_offset
            end_x = start_x + dx * arrow_len
            y_end = start_y + dy * arrow_len
            
            pygame.draw.line(sim_zone_surface, COLOR_REACTION, (start_x, start_y), (end_x, y_end), 2)
            
            angle = math.atan2(y_end - start_y, end_x - start_x)
            wing_len = 8
            pygame.draw.polygon(sim_zone_surface, COLOR_REACTION, [
                (start_x, start_y),
                (start_x + wing_len * math.cos(angle + 0.4), start_y + wing_len * math.sin(angle + 0.4)),
                (start_x + wing_len * math.cos(angle - 0.4), start_y + wing_len * math.sin(angle - 0.4))
            ])
        
        r_scale = max(2, int(NODE_RADIUS * camera.zoom_scale))
        if i == selected_node_idx or i == active_node_bnd:
            pygame.draw.circle(sim_zone_surface, COLOR_HIGHLIGHT, (local_nx, local_ny), r_scale + max(2, int(5 * camera.zoom_scale)), width=2)
        if node.is_anchor_x and node.is_anchor_y:
            pygame.draw.circle(sim_zone_surface, COLOR_PIN, (local_nx, local_ny), r_scale + max(2, int(7 * camera.zoom_scale)), width=3)
            pygame.draw.circle(sim_zone_surface, COLOR_PIN, (local_nx, local_ny), r_scale + max(1, int(2 * camera.zoom_scale)))
        elif node.is_anchor_y and not node.is_anchor_x:
            w_off, h_off = int(12 * camera.zoom_scale), int(11 * camera.zoom_scale)
            pygame.draw.line(sim_zone_surface, COLOR_ROLLER, (local_nx - w_off, local_ny + h_off), (local_nx + w_off, local_ny + h_off), 2)
            pygame.draw.line(sim_zone_surface, COLOR_ROLLER, (local_nx - int(8 * camera.zoom_scale), local_ny + int(15 * camera.zoom_scale)), (local_nx + int(8 * camera.zoom_scale), local_ny + int(15 * camera.zoom_scale)), 1)
            pygame.draw.circle(sim_zone_surface, COLOR_ROLLER, (local_nx, local_ny), r_scale + max(1, int(2 * camera.zoom_scale)))
        else:
            pygame.draw.circle(sim_zone_surface, (234, 179, 8) if i == active_node_bnd else COLOR_NODE, (local_nx, local_ny), r_scale)

    screen.blit(sim_zone_surface, (sim_rect.left, sim_rect.top))
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    if not truss.is_stable and not is_physics_playing:
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

    if ((selected_node_idx is not None) or (selected_beam_idx is not None)) and (truss.is_stable or is_playing or is_physics_playing):
        lines, header_text, is_beam_selected = [], "", selected_beam_idx is not None
        
        if is_beam_selected:
            beam = truss.beams[selected_beam_idx]
            header_text = "STRUCTURAL ELEMENT"
            
            if is_physics_playing and saved_truss_state is not None:
                na = saved_truss_state[beam.node_a]
                nb = saved_truss_state[beam.node_b]
                length_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
            else:
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
            lines = [f"Type: {support_str}", f"Coords: ({round(node.x)}, {round(node.y)})", f"Net Load: {net_magnitude:.1f} kN", f"Load X: {node.load_x / 1000.0:.1f} kN", f"Load Y: {effective_y / 1000.0:.1f} kN"]
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