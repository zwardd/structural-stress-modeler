import pygame
import math
import json
from truss_model import TrussSystem, Node, Beam
from constants import CLICK_TOLERANCE, GRID_SIZE, sim_rect
from serialization import save_project_dialog, load_project_dialog
from materials import MaterialManager
from physics_engine import PhysicsSimulation

def run_determinism_test(base_truss, num_runs=3, frames=180):
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

    diverged = False
    for f in range(frames):
        val = run_histories[0][f]
        for r in range(1, num_runs):
            if run_histories[r][f] != val:
                diverged = True
                break
        if diverged: break
    
def point_to_line_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0: return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

class InputHandler:
    @staticmethod
    def process_events(truss, camera, sim_ctrl, ui_rects, app_state, trigger_status):
        mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                app_state["is_running"] = False
                
            elif event.type == pygame.VIDEORESIZE:
                app_state["request_resize"] = (event.w, event.h)

            elif event.type == pygame.MOUSEWHEEL:
                if camera.process_event(event, mouse_pos):
                    continue

            elif event.type == pygame.KEYDOWN:
                if app_state["input_active"]:
                    if event.key == pygame.K_RETURN:
                        try:
                            val = float(app_state["input_buffer"]) * 1000.0
                            if app_state["selected_node_idx"] is not None:
                                if app_state["input_type"] == "X": 
                                    truss.nodes[app_state["selected_node_idx"]].load_x = val
                                else: 
                                    truss.nodes[app_state["selected_node_idx"]].load_y = -val
                            app_state["first_break_gravity"] = None
                        except ValueError: pass
                        app_state["input_active"] = False
                        app_state["input_buffer"] = ""
                    elif event.key == pygame.K_BACKSPACE: 
                        app_state["input_buffer"] = app_state["input_buffer"][:-1]
                    elif event.unicode in "0123456789.-": 
                        app_state["input_buffer"] += event.unicode
                    continue

                ctrl_pressed = pygame.key.get_pressed()[pygame.K_LCTRL] or pygame.key.get_pressed()[pygame.K_RCTRL]
                if ctrl_pressed and sim_ctrl.state == "EDIT":
                    if event.key == pygame.K_s:
                        status = save_project_dialog(truss)
                        if status: trigger_status(status)
                        continue
                    elif event.key == pygame.K_o:
                        success, status = load_project_dialog(truss)
                        if success:
                            app_state["selected_node_idx"] = None
                            app_state["selected_beam_idx"] = None
                            app_state["active_node_bnd"] = None
                            app_state["gravity_multiplier"] = 0.0
                            app_state["first_break_gravity"] = None
                            app_state["fading_beams"].clear()
                            app_state["show_benchmark_hud"] = False
                            app_state["is_optimizing"] = False
                            camera.reset()
                            trigger_status(status)
                        elif status:
                            trigger_status(status)
                        continue

                if event.key == pygame.K_F11:
                    app_state["is_fullscreen"] = not app_state.get("is_fullscreen", False)
                    app_state["request_fullscreen_toggle"] = True

                elif event.key == pygame.K_SPACE and not pygame.key.get_mods() & pygame.KMOD_ALT:
                    if sim_ctrl.state == "EDIT":
                        sim_ctrl.play(truss, app_state["gravity_multiplier"])
                        app_state["is_optimizing"] = False
                        app_state["active_node_bnd"] = None
                        app_state["input_active"] = False
                        trigger_status("DYNAMIC PLAY MODE ACTIVE")
                    elif sim_ctrl.state == "PLAY":
                        sim_ctrl.pause()
                        trigger_status("SIMULATION PAUSED")
                    elif sim_ctrl.state == "PAUSE":
                        sim_ctrl.play(truss, app_state["gravity_multiplier"])
                        trigger_status("SIMULATION RESUMED")
                elif event.key == pygame.K_t and pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    if sim_ctrl.state == "EDIT":
                        run_determinism_test(truss)
                elif event.key == pygame.K_h:
                    camera.reset()
                    trigger_status("CAMERA RESET TO ORIGIN")
                elif event.key == pygame.K_EQUALS:
                    if app_state["first_break_gravity"] is not None: 
                        app_state["gravity_multiplier"] = min(app_state["first_break_gravity"], app_state["gravity_multiplier"] + 0.5)
                    else: 
                        app_state["gravity_multiplier"] = min(100.0, app_state["gravity_multiplier"] + 0.5)
                elif event.key == pygame.K_MINUS: 
                    app_state["gravity_multiplier"] = max(0.0, app_state["gravity_multiplier"] - 0.5)
                    
                if sim_ctrl.state == "EDIT":
                    if event.key == pygame.K_r:
                        truss.clear()
                        app_state["active_node_bnd"] = None
                        app_state["selected_node_idx"] = None
                        app_state["selected_beam_idx"] = None
                        app_state["gravity_multiplier"] = 0.0
                        app_state["first_break_gravity"] = None
                        app_state["fading_beams"].clear()
                        app_state["input_active"] = False
                        app_state["show_benchmark_hud"] = False
                        app_state["is_optimizing"] = False
                        camera.reset()
                    elif event.key == pygame.K_g: 
                        app_state["grid_enabled"] = not app_state["grid_enabled"]
                    elif event.key == pygame.K_w:
                        truss.self_weight_enabled = not truss.self_weight_enabled
                    elif event.key == pygame.K_v:
                        app_state["show_benchmark_hud"] = not app_state["show_benchmark_hud"]
                        if app_state["show_benchmark_hud"]:
                            truss.load_benchmark_case()
                            app_state["selected_node_idx"] = None
                            app_state["selected_beam_idx"] = None
                            app_state["active_node_bnd"] = None
                            app_state["first_break_gravity"] = None
                            app_state["is_optimizing"] = False
                            camera.reset()
                    elif event.key == pygame.K_1: truss.set_material("Steel")
                    elif event.key == pygame.K_2: truss.set_material("Aluminum")
                    elif event.key == pygame.K_3: truss.set_material("Titanium")
                    elif event.key == pygame.K_p and app_state["selected_beam_idx"] is not None:
                        truss.beams[app_state["selected_beam_idx"]].cycle_profile()
                        app_state["first_break_gravity"] = None
                    elif event.key == pygame.K_LEFTBRACKET and app_state["selected_beam_idx"] is not None:
                        mods = pygame.key.get_mods()
                        b = truss.beams[app_state["selected_beam_idx"]]
                        if mods & pygame.KMOD_SHIFT:
                            if b.profile != "Solid Bar":
                                b.dim_t = max(0.001, b.dim_t - 0.001)
                                b.recalculate_geometry()
                        else:
                            b.dim_w = max(0.01, b.dim_w - 0.005)
                            if b.profile != "Solid Bar" and b.dim_t > b.dim_w * 0.45:
                                b.dim_t = b.dim_w * 0.45
                            b.recalculate_geometry()
                        app_state["first_break_gravity"] = None
                    elif event.key == pygame.K_RIGHTBRACKET and app_state["selected_beam_idx"] is not None:
                        mods = pygame.key.get_mods()
                        b = truss.beams[app_state["selected_beam_idx"]]
                        if mods & pygame.KMOD_SHIFT:
                            if b.profile != "Solid Bar":
                                b.dim_t = min(b.dim_w * 0.45, b.dim_t + 0.001)
                                b.recalculate_geometry()
                        else:
                            b.dim_w = min(0.32, b.dim_w + 0.005)
                            b.recalculate_geometry()
                        app_state["first_break_gravity"] = None
                    elif event.key == pygame.K_m and app_state["selected_beam_idx"] is not None:
                        b = truss.beams[app_state["selected_beam_idx"]]
                        next_m = MaterialManager.get_next_material(b.material)
                        b.update_material_properties(next_m)
                        b.reset_status()
                        app_state["first_break_gravity"] = None
                    elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                        if app_state["selected_beam_idx"] is not None:
                            truss.beams.pop(app_state["selected_beam_idx"])
                            app_state["selected_beam_idx"] = None
                            app_state["first_break_gravity"] = None
                        elif app_state["selected_node_idx"] is not None:
                            idx = app_state["selected_node_idx"]
                            truss.beams = [b for b in truss.beams if b.node_a != idx and b.node_b != idx]
                            for b in truss.beams:
                                if b.node_a > idx: b.node_a -= 1
                                if b.node_b > idx: b.node_b -= 1
                            truss.nodes.pop(idx)
                            app_state["selected_node_idx"] = None
                            app_state["first_break_gravity"] = None
                else:
                    if event.key == pygame.K_w:
                        truss.self_weight_enabled = not truss.self_weight_enabled
                        if sim_ctrl.physics_sim is not None:
                            sim_ctrl.physics_sim.enable_gravity = truss.self_weight_enabled

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if ui_rects["sidebar"].collidepoint(event.pos):
                    if ui_rects["btn_play"].collidepoint(event.pos):
                        if sim_ctrl.state == "EDIT":
                            sim_ctrl.play(truss, app_state["gravity_multiplier"])
                            app_state["is_optimizing"] = False
                            app_state["input_active"] = False
                            trigger_status("DYNAMIC PLAY MODE ACTIVE")
                        elif sim_ctrl.state == "PAUSE":
                            sim_ctrl.play(truss, app_state["gravity_multiplier"])
                            trigger_status("SIMULATION RESUMED")
                    elif ui_rects["btn_pause"].collidepoint(event.pos):
                        if sim_ctrl.state == "PLAY":
                            sim_ctrl.pause()
                            trigger_status("SIMULATION PAUSED")
                    elif ui_rects["btn_reset"].collidepoint(event.pos):
                        if sim_ctrl.state != "EDIT":
                            sim_ctrl.reset(truss)
                            trigger_status("DYNAMIC SIMULATION RESET")
                    elif ui_rects["btn_s25"].collidepoint(event.pos): sim_ctrl.set_speed(0.25)
                    elif ui_rects["btn_s50"].collidepoint(event.pos): sim_ctrl.set_speed(0.5)
                    elif ui_rects["btn_s10"].collidepoint(event.pos): sim_ctrl.set_speed(1.0)
                    elif ui_rects["btn_s20"].collidepoint(event.pos): sim_ctrl.set_speed(2.0)
                    elif ui_rects["btn_s40"].collidepoint(event.pos): sim_ctrl.set_speed(4.0)
                    elif ui_rects["btn_w_toggle"].collidepoint(event.pos):
                        truss.self_weight_enabled = not truss.self_weight_enabled
                        if sim_ctrl.state != "EDIT" and sim_ctrl.physics_sim is not None:
                            sim_ctrl.physics_sim.enable_gravity = truss.self_weight_enabled
                    elif ui_rects["btn_optimize"].collidepoint(event.pos) and sim_ctrl.state == "EDIT":
                        app_state["is_optimizing"] = not app_state["is_optimizing"]
                        if app_state["is_optimizing"]: trigger_status("OPTIMIZATION ENGINE ACTIVE")
                        else: trigger_status("OPTIMIZATION HALTED")
                    elif ui_rects["btn_trails"].collidepoint(event.pos):
                        app_state["trails_enabled"] = not app_state["trails_enabled"]
                    elif ui_rects["chk_profile"].inflate(10, 10).collidepoint(event.pos):
                        app_state["allow_profile_switching"] = not app_state["allow_profile_switching"]
                    elif ui_rects["btn_select"].collidepoint(event.pos) and sim_ctrl.state == "EDIT": 
                        app_state["current_mode"], app_state["input_active"] = "SELECT", False
                    elif ui_rects["btn_node"].collidepoint(event.pos) and sim_ctrl.state == "EDIT": 
                        app_state["current_mode"], app_state["input_active"] = "NODE", False
                    elif ui_rects["btn_beam"].collidepoint(event.pos) and sim_ctrl.state == "EDIT": 
                        app_state["current_mode"], app_state["input_active"] = "BEAM", False
                    elif ui_rects["btn_load"].collidepoint(event.pos) and sim_ctrl.state == "EDIT": 
                        app_state["current_mode"], app_state["input_active"] = "LOAD", False
                    elif ui_rects["btn_benchmark"].collidepoint(event.pos) and sim_ctrl.state == "EDIT":
                        app_state["show_benchmark_hud"] = not app_state["show_benchmark_hud"]
                        if app_state["show_benchmark_hud"]:
                            truss.load_benchmark_case()
                            app_state["selected_node_idx"] = None
                            app_state["selected_beam_idx"] = None
                            app_state["active_node_bnd"] = None
                            app_state["first_break_gravity"] = None
                            app_state["is_optimizing"] = False
                            camera.reset()
                    continue

                if app_state["input_active"]: 
                    app_state["input_active"], app_state["input_buffer"] = False, ""
                    
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
                    if app_state["current_mode"] == "LOAD" and clicked_node_idx is not None: 
                        app_state["selected_node_idx"] = clicked_node_idx
                        app_state["selected_beam_idx"] = None
                    elif app_state["current_mode"] == "SELECT":
                        app_state["selected_node_idx"] = clicked_node_idx
                        app_state["selected_beam_idx"] = None
                        if clicked_node_idx is None:
                            for i, b in enumerate(truss.beams):
                                if b.status == "FRACTURED": continue
                                if point_to_line_distance(sim_mouse_x, sim_mouse_y, truss.nodes[b.node_a].x, truss.nodes[b.node_a].y, truss.nodes[b.node_b].x, truss.nodes[b.node_b].y) < (8 / camera.zoom_scale):
                                    app_state["selected_beam_idx"] = i
                    elif app_state["current_mode"] == "NODE":
                        sim_mouse_x = max(-camera.WORKSPACE_LIMIT, min(camera.WORKSPACE_LIMIT, sim_mouse_x))
                        sim_mouse_y = max(-camera.WORKSPACE_LIMIT, min(camera.WORKSPACE_LIMIT, sim_mouse_y))
                        truss.add_node(sim_mouse_x, sim_mouse_y, snap_enabled=app_state["grid_enabled"], grid_size=GRID_SIZE)
                        app_state["first_break_gravity"] = None
                        app_state["show_benchmark_hud"] = False
                    elif app_state["current_mode"] == "BEAM":
                        if clicked_node_idx is not None:
                            if app_state["active_node_bnd"] is None: 
                                app_state["active_node_bnd"] = clicked_node_idx
                            else:
                                truss.add_beam(app_state["active_node_bnd"], clicked_node_idx)
                                app_state["active_node_bnd"] = None
                                app_state["first_break_gravity"] = None
                                app_state["show_benchmark_hud"] = False
                        else: 
                            app_state["active_node_bnd"] = None
                elif event.button == 3 and clicked_node_idx is not None:
                    truss.nodes[clicked_node_idx].toggle_support()
                    app_state["first_break_gravity"] = None
                    app_state["show_benchmark_hud"] = False

            elif event.type == pygame.MOUSEBUTTONUP:
                camera.process_event(event, mouse_pos)

    @staticmethod
    def process_continuous(truss, sim_ctrl, app_state):
        if app_state["selected_node_idx"] is not None and app_state["current_mode"] == "SELECT" and sim_ctrl.state == "EDIT" and not app_state["input_active"]:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP] or keys[pygame.K_DOWN] or keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]:
                app_state["first_break_gravity"] = None
                app_state["show_benchmark_hud"] = False
            if keys[pygame.K_UP]: truss.nodes[app_state["selected_node_idx"]].load_y -= 1000.0
            if keys[pygame.K_DOWN]: truss.nodes[app_state["selected_node_idx"]].load_y += 1000.0
            if keys[pygame.K_LEFT]: truss.nodes[app_state["selected_node_idx"]].load_x -= 1000.0
            if keys[pygame.K_RIGHT]: truss.nodes[app_state["selected_node_idx"]].load_x += 1000.0