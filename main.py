import pygame
import sys
import math
import tkinter as tk
from truss_model import TrussSystem
from matrix_solver import solve_truss
from constants import *
from camera import Camera
from materials import MaterialManager
from physics_engine import PhysicsSimulation
from simulation_controller import SimulationController
from drawing_manager import find_proxy_limit, get_stress_color, draw_force_vector, draw_curved_beam, draw_cable_element, draw_road_element
from ui_panels import UIManager
from input_handler import InputHandler

root = tk.Tk()
root.withdraw()

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

font_header = pygame.font.SysFont("Helvetica", 16, bold=True)
font_body = pygame.font.SysFont("Helvetica", 13)

clock = pygame.time.Clock()
truss = TrussSystem()
ui_manager = UIManager(font_header, font_body)
camera = Camera()
sim_ctrl = SimulationController()

app_state = {
    "is_running": True,
    "current_mode": "SELECT",
    "is_optimizing": False,
    "allow_profile_switching": True,
    "grid_enabled": True,
    "trails_enabled": False,
    "gravity_multiplier": 0.0,
    "first_break_gravity": None,
    "show_benchmark_hud": False,
    "input_buffer": "",
    "input_active": False,
    "input_type": "Y",
    "fading_beams": [],
    "active_node_bnd": None,
    "selected_node_idx": None,
    "selected_beam_idx": None,
    "selected_cable_idx": None,
    "selected_road_idx": None,
    "is_fullscreen": False,
    "windowed_size": (WINDOW_WIDTH, WINDOW_HEIGHT),
    "request_resize": None,
    "request_fullscreen_toggle": False
}

status_banner_text = ""
status_banner_timer = 0

def trigger_status(text):
    global status_banner_text, status_banner_timer
    status_banner_text = text
    status_banner_timer = 180  

def calculate_utilization(elem):
    if elem.status == "FRACTURED" or elem.force == 0.0:
        return 0.0
        
    is_cable = getattr(elem, "is_cable", False)
    if is_cable and elem.force <= 1e-4:
        return 0.0

    is_road = getattr(elem, "is_road", False)
    if is_road:
        yield_stress = elem.modulus * 0.005 
        return abs(elem.stress) / yield_stress if yield_stress > 0 else 0
        
    yield_stress = MaterialManager.get_yield_stress(elem.material)
    yield_util = abs(elem.stress) / yield_stress
    
    if elem.status == "YIELDING" and not is_cable and not is_road:
        limit, proxy_node = find_proxy_limit(elem, truss)
        if proxy_node is not None:
            current_visual_bow = min(35.0, (yield_util - 0.95) * 18.0)
            if current_visual_bow >= limit:
                return max(yield_util * 1.6, 1.2)
                
    if elem.stress < -1e-2 and not is_cable and not is_road:
        length_m = truss.get_beam_length(elem)
        if length_m > 0:
            p_crit = MaterialManager.calculate_buckling_load(elem.material, elem.inertia, length_m)
            buckle_util = abs(elem.force) / p_crit
            return max(yield_util, buckle_util)
            
    return yield_util

def run_dynamic_eval(truss_obj, grav_mult):
    saved_state = [{"x": n.x, "y": n.y} for n in truss_obj.nodes]
    truss_obj.reset_sim_stats()
    
    sim = PhysicsSimulation(truss_obj, gravity_mult=grav_mult, enable_gravity=truss_obj.self_weight_enabled)
    
    frames = 120 
    for _ in range(frames):
        truss_obj.sim_stats["sim_time"] += 1.0 / 60.0
        sim.step(grav_mult)
        sim.sync_to_truss(truss_obj)
        
        for elem in truss_obj.beams + truss_obj.cables + truss_obj.roads:
            if elem.status == "FRACTURED": continue
            util = calculate_utilization(elem)
            fos = 1.0 / util if util > 0.0 else float('inf')
            elem.peak_utilization_seen = max(elem.peak_utilization_seen, util)
            elem.minimum_fos_seen = min(elem.minimum_fos_seen, fos)
            
            truss_obj.peak_utilization_recorded = max(truss_obj.peak_utilization_recorded, util)
            truss_obj.minimum_fos_recorded = min(truss_obj.minimum_fos_recorded, fos)
            
    for i, s in enumerate(saved_state):
        truss_obj.nodes[i].x = s["x"]
        truss_obj.nodes[i].y = s["y"]

def compute_dynamic_reactions(truss, gravity_mult):
    g = 9.81 * gravity_mult
    for node in truss.nodes:
        node.rx = 0.0
        node.ry = 0.0
    for i, node in enumerate(truss.nodes):
        if not node.is_anchor_x and not node.is_anchor_y: continue
        fx_net = node.load_x
        fy_net = node.load_y
        
        if truss.self_weight_enabled and gravity_mult > 0.0:
            for elem in truss.beams + truss.cables + truss.roads:
                if elem.status == "FRACTURED": continue
                if elem.node_a == i or elem.node_b == i:
                    L_m = truss.get_beam_length(elem)
                    if sim_ctrl.state != "EDIT" and sim_ctrl.saved_truss_state is not None:
                        na = sim_ctrl.saved_truss_state[elem.node_a]
                        nb = sim_ctrl.saved_truss_state[elem.node_b]
                        L_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
                    fy_net += (L_m * elem.area * elem.density * g) / 2.0
                    
        for elem in truss.beams + truss.cables + truss.roads:
            if elem.status == "FRACTURED": continue
            if elem.node_a == i or elem.node_b == i:
                na = truss.nodes[elem.node_a]
                nb = truss.nodes[elem.node_b]
                dx = nb.x - na.x
                dy = nb.y - na.y
                L_px = math.hypot(dx, dy)
                if L_px > 1e-6:
                    dir_x = dx / L_px
                    dir_y = dy / L_px
                    if elem.node_a == i:
                        fx_net += elem.force * dir_x
                        fy_net += elem.force * dir_y
                    else:
                        fx_net -= elem.force * dir_x
                        fy_net -= elem.force * dir_y
                        
        if node.is_anchor_x: node.rx = -fx_net
        if node.is_anchor_y: node.ry = -fy_net

def build_ui_rects(w, h):
    return {
        "sidebar": pygame.Rect(0, 0, 140, h),
        "btn_select": pygame.Rect(15, 80, 110, 35), 
        "btn_node": pygame.Rect(15, 125, 110, 35), 
        "btn_beam": pygame.Rect(15, 170, 110, 35), 
        "btn_cable": pygame.Rect(15, 215, 110, 35), 
        "btn_road": pygame.Rect(15, 260, 110, 35),
        "btn_load": pygame.Rect(15, 305, 110, 35), 
        "btn_benchmark": pygame.Rect(15, 350, 110, 35),
        "btn_play": pygame.Rect(15, h - 220, 34, 30), 
        "btn_pause": pygame.Rect(53, h - 220, 34, 30), 
        "btn_reset": pygame.Rect(91, h - 220, 34, 30),
        "btn_s25": pygame.Rect(15 + 0*22, h - 170, 20, 20), 
        "btn_s50": pygame.Rect(15 + 1*22, h - 170, 20, 20), 
        "btn_s10": pygame.Rect(15 + 2*22, h - 170, 20, 20), 
        "btn_s20": pygame.Rect(15 + 3*22, h - 170, 20, 20), 
        "btn_s40": pygame.Rect(15 + 4*22, h - 170, 20, 20),
        "btn_w_toggle": pygame.Rect(15, h - 140, 110, 30), 
        "btn_optimize": pygame.Rect(15, h - 100, 110, 35), 
        "btn_trails": pygame.Rect(15, h - 55, 110, 30), 
        "chk_profile": pygame.Rect(15, h - 15, 14, 14)
    }

ui_rects = build_ui_rects(WINDOW_WIDTH, WINDOW_HEIGHT)

while app_state["is_running"]:
    if app_state.get("request_resize"):
        w, h = app_state["request_resize"]
        is_fs = app_state["is_fullscreen"]
        screen = pygame.display.set_mode((w, h), pygame.RESIZABLE | (pygame.FULLSCREEN if is_fs else 0))
        sim_rect.update(160, 20, w - 180, h - 40)
        ui_rects = build_ui_rects(w, h)
        app_state["request_resize"] = None
        
    if app_state.get("request_fullscreen_toggle"):
        is_fs = app_state["is_fullscreen"]
        if is_fs:
            app_state["windowed_size"] = (screen.get_width(), screen.get_height())
            info = pygame.display.Info()
            w, h = info.current_w, info.current_h
            screen = pygame.display.set_mode((w, h), pygame.FULLSCREEN)
        else:
            w, h = app_state.get("windowed_size", (WINDOW_WIDTH, WINDOW_HEIGHT))
            screen = pygame.display.set_mode((w, h), pygame.RESIZABLE)
            
        sim_rect.update(160, 20, w - 180, h - 40)
        ui_rects = build_ui_rects(w, h)
        app_state["request_fullscreen_toggle"] = False

    InputHandler.process_events(truss, camera, sim_ctrl, ui_rects, app_state, trigger_status)
    InputHandler.process_continuous(truss, sim_ctrl, app_state)
    camera.update(pygame.mouse.get_pos())

    if sim_ctrl.state in ["PLAY", "PAUSE"]:
        if sim_ctrl.state == "PLAY" and sim_ctrl.physics_sim is not None:
            sim_ctrl.time_accumulator += sim_ctrl.speed
            
            while sim_ctrl.time_accumulator >= 1.0:
                truss.sim_stats["sim_time"] += 1.0 / 60.0
                sim_ctrl.physics_sim.step(app_state["gravity_multiplier"])
                sim_ctrl.physics_sim.sync_to_truss(truss)
                
                for node in truss.nodes:
                    node.trail.append((node.x, node.y))
                    if len(node.trail) > 200: node.trail.pop(0)
                        
                compute_dynamic_reactions(truss, app_state["gravity_multiplier"])

                beams_to_break = []
                current_mass = 0.0
                current_max_util = 0.0
                
                for e_idx, elem in enumerate(truss.beams + truss.cables + truss.roads):
                    if elem.status == "FRACTURED":
                        elem.history.append(0.0)
                        if len(elem.history) > 300: elem.history.pop(0)
                        continue
                    
                    is_road = getattr(elem, "is_road", False)    
                    if is_road:
                        ultimate_stress = elem.modulus * 0.01 
                    else:
                        ultimate_stress = MaterialManager.get_ultimate_stress(elem.material)
                    
                    utilization = calculate_utilization(elem)
                    
                    elem.history.append(utilization * 100.0)
                    if len(elem.history) > 300: elem.history.pop(0)
                    
                    na = truss.nodes[elem.node_a]
                    nb = truss.nodes[elem.node_b]
                    length_m = math.hypot(nb.x - na.x, nb.y - na.y) * 0.0125
                    current_mass += length_m * elem.area * elem.density
                    
                    fos = 1.0 / utilization if utilization > 0.0 else float('inf')
                    elem.peak_utilization_seen = max(elem.peak_utilization_seen, utilization)
                    elem.minimum_fos_seen = min(elem.minimum_fos_seen, fos)
                    current_max_util = max(current_max_util, utilization)
                    
                    if abs(elem.stress) >= ultimate_stress or utilization >= 1.6:
                        beams_to_break.append((elem, utilization, e_idx))

                truss.sim_stats["peak_mass"] = max(truss.sim_stats["peak_mass"], current_mass)
                truss.peak_utilization_recorded = max(truss.peak_utilization_recorded, current_max_util)
                sys_fos = 1.0 / current_max_util if current_max_util > 0.0 else float('inf')
                truss.minimum_fos_recorded = min(truss.minimum_fos_recorded, sys_fos)

                beams_to_break.sort(key=lambda item: (item[1], -item[2]), reverse=True)

                for elem, util, e_idx in beams_to_break:
                    elem.status = "FRACTURED"
                    elem.is_broken = True
                    elem.broken_at_gravity = app_state["gravity_multiplier"]
                    
                    if sim_ctrl.physics_sim is not None:
                        sim_ctrl.physics_sim.remove_constraints_for_beam(elem)
                    if app_state["first_break_gravity"] is None:
                        app_state["first_break_gravity"] = app_state["gravity_multiplier"]
                        
                    ax, ay = truss.nodes[elem.node_a].x, truss.nodes[elem.node_a].y
                    bx, by = truss.nodes[elem.node_b].x, truss.nodes[elem.node_b].y
                    mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
                    thick = max(2, int(5 * camera.zoom_scale))
                    app_state["fading_beams"].append([ax, ay, mx - 2, my, thick, 1.0, -1.0])
                    app_state["fading_beams"].append([mx + 2, my, bx, by, thick, 1.0, -1.5])
                    
                for elem in truss.beams + truss.cables + truss.roads:
                    if elem.status == "FRACTURED": continue
                    utilization = calculate_utilization(elem)
                    if utilization >= 1.0 and elem.status == "NORMAL":
                        elem.status = "YIELDING"
                        elem.broken_at_gravity = app_state["gravity_multiplier"]
                    elif utilization < 1.0 and elem.status == "YIELDING":
                        elem.status = "NORMAL"

                sim_ctrl.time_accumulator -= 1.0
    else:
        app_state["first_break_gravity"] = None
        if app_state["is_optimizing"]:
            if len(truss.beams) + len(truss.cables) + len(truss.roads) == 0:
                app_state["is_optimizing"] = False
                trigger_status("OPTIMIZATION FAILED: INVALID STRUCTURE")
            else:
                changed = truss.optimize_step(calculate_utilization, solve_truss, app_state["gravity_multiplier"], app_state["allow_profile_switching"], run_dynamic_eval)
                if not changed:
                    app_state["is_optimizing"] = False
                    trigger_status("OPTIMIZATION COMPLETE (FOS > 2.0 ACHIEVED)")
                else:
                    pygame.time.delay(90)
        else:
            for elem in truss.beams + truss.cables + truss.roads:
                elem.reset_status()
            solve_truss(truss, 0.0)

    for fb in app_state["fading_beams"][:]:
        fb[5] -= 0.04  
        fb[6] += 0.45  
        fb[1] += fb[6]
        fb[3] += fb[6]
        if fb[5] <= 0: app_state["fading_beams"].remove(fb)

    screen.fill(COLOR_BACKGROUND)
    
    ui_manager.draw_sidebar(
        screen, truss, sim_ctrl, app_state["current_mode"], app_state["show_benchmark_hud"], 
        app_state["is_optimizing"], app_state["trails_enabled"], app_state["allow_profile_switching"], 
        ui_rects, calculate_utilization
    )

    sim_zone_surface = pygame.Surface((sim_rect.width, sim_rect.height))
    sim_zone_surface.fill(COLOR_SIM_ZONE)

    if app_state["grid_enabled"]:
        scaled_grid = GRID_SIZE * camera.zoom_scale
        if scaled_grid > 2:
            start_sim_x, start_sim_y = camera.to_sim(sim_rect.left, sim_rect.top)
            end_sim_x, end_sim_y = camera.to_sim(sim_rect.right, sim_rect.bottom)

            sim_left = min(start_sim_x, end_sim_x)
            sim_right = max(start_sim_x, end_sim_x)
            sim_top = min(start_sim_y, end_sim_y)
            sim_bottom = max(start_sim_y, end_sim_y)

            start_g_x = (math.ceil(sim_left / GRID_SIZE) * GRID_SIZE)
            start_g_y = (math.ceil(sim_top / GRID_SIZE) * GRID_SIZE)

            curr_g_x = start_g_x
            while curr_g_x <= sim_right:
                screen_x, _ = camera.to_screen(curr_g_x, 0)
                local_x = screen_x - sim_rect.left
                if 0 <= local_x <= sim_rect.width:
                    pygame.draw.line(sim_zone_surface, COLOR_GRID, (local_x, 0), (local_x, sim_rect.height))
                curr_g_x += GRID_SIZE

            curr_g_y = start_g_y
            while curr_g_y <= sim_bottom:
                _, screen_y = camera.to_screen(0, curr_g_y)
                local_y = screen_y - sim_rect.top
                if 0 <= local_y <= sim_rect.height:
                    pygame.draw.line(sim_zone_surface, COLOR_GRID, (0, local_y), (sim_rect.width, local_y))
                curr_g_y += GRID_SIZE

    node_has_connections = [False] * len(truss.nodes)
    for elem in truss.beams + truss.cables + truss.roads:
        if elem.status != "FRACTURED":
            node_has_connections[elem.node_a] = True
            node_has_connections[elem.node_b] = True

    for i, road in enumerate(truss.roads):
        if road.status == "FRACTURED": continue
        ax, ay = truss.nodes[road.node_a].x, truss.nodes[road.node_a].y
        bx, by = truss.nodes[road.node_b].x, truss.nodes[road.node_b].y

        sx, sy = camera.to_screen(ax, ay)
        local_ax = round(sx-sim_rect.left)
        local_ay = round(sy-sim_rect.top)
        sx, sy = camera.to_screen(bx, by)
        local_bx = round(sx-sim_rect.left)
        local_by = round(sy-sim_rect.top)

        thickness_pixels = max(2, int(5 * camera.zoom_scale))
        draw_road_element(sim_zone_surface, local_ax, local_ay, local_bx, local_by, road, thickness_pixels, get_stress_color(road, truss), i == app_state["selected_road_idx"])

    for i, beam in enumerate(truss.beams):
        if beam.status == "FRACTURED": continue
        ax, ay = truss.nodes[beam.node_a].x, truss.nodes[beam.node_a].y
        bx, by = truss.nodes[beam.node_b].x, truss.nodes[beam.node_b].y

        sx, sy = camera.to_screen(ax, ay)
        local_ax = round(sx-sim_rect.left)
        local_ay = round(sy-sim_rect.top)
        sx, sy = camera.to_screen(bx, by)
        local_bx = round(sx-sim_rect.left)
        local_by = round(sy-sim_rect.top)

        thickness_pixels = max(2, int(5 * camera.zoom_scale))
        draw_curved_beam(sim_zone_surface, local_ax, local_ay, local_bx, local_by, beam, thickness_pixels, get_stress_color(beam, truss), i == app_state["selected_beam_idx"], camera.zoom_scale, truss)

    for i, cable in enumerate(truss.cables):
        if cable.status == "FRACTURED": continue
        ax, ay = truss.nodes[cable.node_a].x, truss.nodes[cable.node_a].y
        bx, by = truss.nodes[cable.node_b].x, truss.nodes[cable.node_b].y

        sx, sy = camera.to_screen(ax, ay)
        local_ax = round(sx-sim_rect.left)
        local_ay = round(sy-sim_rect.top)
        sx, sy = camera.to_screen(bx, by)
        local_bx = round(sx-sim_rect.left)
        local_by = round(sy-sim_rect.top)

        thickness_pixels = max(2, int(5 * camera.zoom_scale))
        draw_cable_element(sim_zone_surface, local_ax, local_ay, local_bx, local_by, cable, thickness_pixels, get_stress_color(cable, truss), i == app_state["selected_cable_idx"], camera.zoom_scale, truss, sim_ctrl)

    for fb in app_state["fading_beams"]:
        alpha = int(fb[5] * 255)
        if alpha <= 0: continue
        rax, ray = camera.to_screen(fb[0], fb[1])
        rbx, rby = camera.to_screen(fb[2], fb[3])

    if app_state["trails_enabled"]:
        trail_thickness = max(1, int(1.5 * camera.zoom_scale))
        for node in truss.nodes:
            if len(node.trail) > 1:
                for t_idx in range(len(node.trail) - 1):
                    p1 = node.trail[t_idx]
                    p2 = node.trail[t_idx+1]
                    sx1, sy1 = camera.to_screen(p1[0], p1[1])
                    sx2, sy2 = camera.to_screen(p2[0], p2[1])
                    lx1 = round(sx1 - sim_rect.left)
                    ly1 = round(sy1 - sim_rect.top)
                    lx2 = round(sx2 - sim_rect.left)
                    ly2 = round(sy2 - sim_rect.top)
                    ratio = (t_idx + 1) / len(node.trail)
                    r = int(33 + (59 - 33) * ratio)
                    g = int(33 + (130 - 33) * ratio)
                    b = int(38 + (246 - 38) * ratio)
                    pygame.draw.line(sim_zone_surface, (r, g, b), (lx1, ly1), (lx2, ly2), trail_thickness)

    for i, node in enumerate(truss.nodes):
        if sim_ctrl.state != "EDIT" and not node_has_connections[i] and not node.is_anchor_x and not node.is_anchor_y: continue
        nx, ny = node.x, node.y
       
        sx, sy = camera.to_screen(nx, ny)
        local_nx = round(sx-sim_rect.left)
        local_ny = round(sy-sim_rect.top)

        total_fx = node.load_x
        total_fy = node.load_y + (1000.0 * app_state["gravity_multiplier"] if truss.self_weight_enabled and sim_ctrl.state != "EDIT" else 0.0)
        
        if truss.self_weight_enabled and sim_ctrl.state != "EDIT" and app_state["gravity_multiplier"] > 0.0:
            g = 9.81 * app_state["gravity_multiplier"]
            for elem in truss.beams + truss.cables + truss.roads:
                if elem.status == "FRACTURED": continue
                if elem.node_a == i or elem.node_b == i:
                    total_fy += (truss.get_beam_length(elem) * elem.area * elem.density * g) / 2.0
            
        draw_force_vector(sim_zone_surface, local_nx, local_ny, total_fx, total_fy, camera.zoom_scale, COLOR_LOAD)
        if sim_ctrl.state != "EDIT" and (abs(node.rx) > 0.1 or abs(node.ry) > 0.1):
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
        if i == app_state["selected_node_idx"] or i == app_state["active_node_bnd"]:
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
            pygame.draw.circle(sim_zone_surface, (234, 179, 8) if i == app_state["active_node_bnd"] else COLOR_NODE, (local_nx, local_ny), r_scale)

    screen.blit(sim_zone_surface, (sim_rect.left, sim_rect.top))
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    if app_state["show_benchmark_hud"] and truss.is_stable:
        ui_manager.draw_benchmark_hud(screen, truss)

    if ((app_state["selected_node_idx"] is not None) or (app_state["selected_beam_idx"] is not None) or (app_state["selected_cable_idx"] is not None) or (app_state["selected_road_idx"] is not None)):
        app_state["input_active"], app_state["input_type"], app_state["input_buffer"] = ui_manager.draw_selection_hud(
            screen, truss, sim_ctrl, app_state["selected_node_idx"], app_state["selected_beam_idx"], app_state["selected_cable_idx"], app_state["selected_road_idx"],
            app_state["current_mode"], app_state["input_active"], app_state["input_type"], app_state["input_buffer"], calculate_utilization
        )

    if status_banner_timer > 0:
        status_banner_timer -= 1
        ui_manager.draw_status_banner(screen, status_banner_text)

    ui_manager.draw_bottom_legend(screen, app_state["gravity_multiplier"], app_state["first_break_gravity"], app_state["grid_enabled"])

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()