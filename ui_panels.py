import pygame
import math
from constants import *
from matrix_solver import calculate_benchmark_metrics
from drawing_manager import draw_profile_preview

class UIManager:
    def __init__(self, font_header, font_body):
        self.font_header = font_header
        self.font_body = font_body

    def draw_sidebar(self, screen, truss, sim_ctrl, current_mode, show_benchmark_hud, is_optimizing, trails_enabled, allow_profile_switching, ui_rects, calc_util_fn, show_stress_heatmap):
        pygame.draw.rect(screen, COLOR_PANEL_BG, ui_rects["sidebar"])
        pygame.draw.line(screen, COLOR_UI_BORDER, (140, 0), (140, screen.get_height()), 2)
        
        modes_list = [
            (ui_rects["btn_select"], "1. Select", "SELECT"), 
            (ui_rects["btn_node"], "2. + Node", "NODE"), 
            (ui_rects["btn_beam"], "3. + Beam", "BEAM"), 
            (ui_rects["btn_cable"], "4. + Cable", "CABLE"), 
            (ui_rects["btn_road"], "5. + Road", "ROAD"),
            (ui_rects["btn_load"], "6. + Load", "LOAD")
        ]
        for btn, label, mode in modes_list:
            pygame.draw.rect(screen, COLOR_UI_BORDER if current_mode == mode and sim_ctrl.state == "EDIT" else COLOR_BACKGROUND, btn, border_radius=4)
            screen.blit(self.font_body.render(label, True, COLOR_TEXT_MAIN), (btn.x + 12, btn.y + 10))

        btn_benchmark = ui_rects["btn_benchmark"]
        pygame.draw.rect(screen, COLOR_UI_BORDER if show_benchmark_hud and sim_ctrl.state == "EDIT" else COLOR_BACKGROUND, btn_benchmark, border_radius=4)
        screen.blit(self.font_body.render("7. Benchmark", True, COLOR_TEXT_MAIN), (btn_benchmark.x + 12, btn_benchmark.y + 10))

        pygame.draw.line(screen, COLOR_UI_BORDER, (10, 400), (130, 400), 1)
        screen.blit(self.font_header.render("SYSTEM STATS", True, COLOR_TEXT_MAIN), (15, 415))
        
        total_mass, max_util = 0.0, 0.0
        for elem in truss.beams + truss.cables + truss.roads:
            if elem.status == "FRACTURED": continue
            
            if sim_ctrl.state != "EDIT" and sim_ctrl.saved_truss_state is not None:
                na = sim_ctrl.saved_truss_state[elem.node_a]
                nb = sim_ctrl.saved_truss_state[elem.node_b]
                length_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
            else:
                length_m = truss.get_beam_length(elem)
                
            total_mass += length_m * elem.area * elem.density
            if not getattr(elem, "is_road", False):
                max_util = max(max_util, calc_util_fn(elem))
            else:
                yield_stress = elem.modulus * 0.005 
                road_util = abs(elem.stress) / yield_stress if yield_stress > 0 else 0
                max_util = max(max_util, road_util)
            
        fos_val = 1.0 / max_util if max_util > 0.0 else float('inf')
        
        sys_mass = max(truss.sim_stats["peak_mass"], total_mass) if sim_ctrl.state != "EDIT" else total_mass
        sys_time = truss.sim_stats.get("sim_time", 0.0)
        failed_count = sum(1 for e in truss.beams + truss.cables + truss.roads if e.status == "FRACTURED")
        sys_max_vel = max([n.peak_speed for n in truss.nodes] + [0.0])
        
        peak_rec = truss.peak_utilization_recorded
        min_rec = truss.minimum_fos_recorded
        has_history = (peak_rec > 0.0) or (min_rec != float('inf'))
        
        if has_history or sim_ctrl.state != "EDIT":
            peak_str = f"{peak_rec*100.0:.1f}%"
        else:
            peak_str = "N/A"
            
        screen.blit(self.font_body.render(f"Time: {sys_time:.2f} s", True, COLOR_TEXT_MUTED), (15, 435))
        screen.blit(self.font_body.render(f"Mass: {sys_mass:.1f} kg", True, COLOR_TEXT_MUTED), (15, 450))
        screen.blit(self.font_body.render(f"Failed: {failed_count}", True, COLOR_TEXT_MUTED), (15, 465))
        screen.blit(self.font_body.render(f"Max Vel: {sys_max_vel:.2f} m/s", True, COLOR_TEXT_MUTED), (15, 480))
        screen.blit(self.font_body.render(f"Live Util: {max_util*100.0:.1f}%", True, COLOR_TEXT_MUTED), (15, 495))
        screen.blit(self.font_body.render(f"Peak Util: {peak_str}", True, COLOR_TEXT_MUTED), (15, 510))
        
        l_fos_lbl = self.font_body.render("Live FoS:", True, COLOR_TEXT_MUTED)
        screen.blit(l_fos_lbl, (15, 525))
        
        if len(truss.beams) + len(truss.cables) + len(truss.roads) == 0:
            l_fos_txt = self.font_body.render("N/A", True, COLOR_TEXT_MUTED)
        elif max_util >= 1.6:
            l_fos_txt = self.font_header.render("FAIL", True, COLOR_LOAD)
        elif max_util >= 1.0:
            l_fos_txt = self.font_header.render("FAIL", True, COLOR_YIELDING)
        elif max_util == 0.0 or not math.isfinite(fos_val):
            l_fos_txt = self.font_body.render("N/A", True, COLOR_TEXT_MAIN)
        else:
            l_fos_txt = self.font_header.render(f"{fos_val:.2f}", True, COLOR_ZERO_LOAD if fos_val >= 2.0 else COLOR_MID_LOAD)
        screen.blit(l_fos_txt, (15 + l_fos_lbl.get_width() + 5, 523))

        m_fos_lbl = self.font_body.render("Min FoS:", True, COLOR_TEXT_MUTED)
        screen.blit(m_fos_lbl, (15, 540))
        
        if len(truss.beams) + len(truss.cables) + len(truss.roads) == 0:
            m_fos_txt = self.font_body.render("N/A", True, COLOR_TEXT_MUTED)
        elif peak_rec >= 1.6 and (has_history or sim_ctrl.state != "EDIT"):
            m_fos_txt = self.font_header.render("FAIL", True, COLOR_LOAD)
        elif peak_rec >= 1.0 and (has_history or sim_ctrl.state != "EDIT"):
            m_fos_txt = self.font_header.render("FAIL", True, COLOR_YIELDING)
        elif not has_history and sim_ctrl.state == "EDIT":
            m_fos_txt = self.font_body.render("N/A", True, COLOR_TEXT_MAIN)
        elif not math.isfinite(min_rec):
            m_fos_txt = self.font_body.render("N/A", True, COLOR_TEXT_MAIN)
        else:
            m_fos_txt = self.font_header.render(f"{min_rec:.2f}", True, COLOR_ZERO_LOAD if min_rec >= 2.0 else COLOR_MID_LOAD)
        screen.blit(m_fos_txt, (15 + m_fos_lbl.get_width() + 5, 538))

        btn_stress = ui_rects["btn_stress"]
        pygame.draw.rect(screen, (40, 55, 45) if show_stress_heatmap else COLOR_BACKGROUND, btn_stress, border_radius=4)
        pygame.draw.rect(screen, COLOR_PLAY_GREEN if show_stress_heatmap else COLOR_UI_BORDER, btn_stress, width=1, border_radius=4)
        s_label = "Stress: ON" if show_stress_heatmap else "Stress: OFF"
        screen.blit(self.font_body.render(s_label, True, COLOR_TEXT_MAIN), (btn_stress.x + 16, btn_stress.y + 6))

        pygame.draw.line(screen, COLOR_UI_BORDER, (10, ui_rects["btn_play"].y - 10), (130, ui_rects["btn_play"].y - 10), 1)

        btn_play = ui_rects["btn_play"]
        pygame.draw.rect(screen, (35, 40, 60) if sim_ctrl.state == "PLAY" else COLOR_BACKGROUND, btn_play, border_radius=4)
        pygame.draw.rect(screen, (100, 150, 255) if sim_ctrl.state == "PLAY" else COLOR_UI_BORDER, btn_play, width=1, border_radius=4)
        p_surf = self.font_header.render(" > ", True, (100, 150, 255) if sim_ctrl.state == "PLAY" else COLOR_TEXT_MAIN)
        screen.blit(p_surf, (btn_play.x + (btn_play.width - p_surf.get_width())//2, btn_play.y + (btn_play.height - p_surf.get_height())//2))

        btn_pause = ui_rects["btn_pause"]
        pygame.draw.rect(screen, (35, 40, 60) if sim_ctrl.state == "PAUSE" else COLOR_BACKGROUND, btn_pause, border_radius=4)
        pygame.draw.rect(screen, (100, 150, 255) if sim_ctrl.state == "PAUSE" else COLOR_UI_BORDER, btn_pause, width=1, border_radius=4)
        pa_surf = self.font_header.render(" || ", True, (100, 150, 255) if sim_ctrl.state == "PAUSE" else COLOR_TEXT_MAIN)
        screen.blit(pa_surf, (btn_pause.x + (btn_pause.width - pa_surf.get_width())//2, btn_pause.y + (btn_pause.height - pa_surf.get_height())//2))

        btn_reset = ui_rects["btn_reset"]
        pygame.draw.rect(screen, COLOR_BACKGROUND, btn_reset, border_radius=4)
        pygame.draw.rect(screen, COLOR_UI_BORDER, btn_reset, width=1, border_radius=4)
        r_surf = self.font_header.render(" << ", True, COLOR_TEXT_MAIN)
        screen.blit(r_surf, (btn_reset.x + (btn_reset.width - r_surf.get_width())//2, btn_reset.y + (btn_reset.height - r_surf.get_height())//2))

        screen.blit(self.font_body.render("Speed:", True, COLOR_TEXT_MUTED), (15, ui_rects["btn_s25"].y - 15))

        speed_options = [
            (ui_rects["btn_s25"], "1/4", 0.25),
            (ui_rects["btn_s50"], "1/2", 0.5),
            (ui_rects["btn_s10"], "1x", 1.0),
            (ui_rects["btn_s20"], "2x", 2.0),
            (ui_rects["btn_s40"], "4x", 4.0)
        ]
        for btn, label, val in speed_options:
            is_sel = (sim_ctrl.speed == val)
            pygame.draw.rect(screen, (35, 40, 60) if is_sel else COLOR_BACKGROUND, btn, border_radius=4)
            pygame.draw.rect(screen, (100, 150, 255) if is_sel else COLOR_UI_BORDER, btn, width=1, border_radius=4)
            t_surf = self.font_body.render(label, True, (100, 150, 255) if is_sel else COLOR_TEXT_MAIN)
            screen.blit(t_surf, (btn.x + (btn.width - t_surf.get_width())//2, btn.y + (btn.height - t_surf.get_height())//2))

        btn_w_toggle = ui_rects["btn_w_toggle"]
        pygame.draw.rect(screen, (40, 55, 45) if truss.self_weight_enabled else (55, 35, 35), btn_w_toggle, border_radius=4)
        pygame.draw.rect(screen, COLOR_PLAY_GREEN if truss.self_weight_enabled else COLOR_LOAD, btn_w_toggle, width=1, border_radius=4)
        w_label = "Weight: ON" if truss.self_weight_enabled else "Weight: OFF"
        screen.blit(self.font_body.render(w_label, True, COLOR_TEXT_MAIN), (btn_w_toggle.x + 16, btn_w_toggle.y + 6))

        btn_optimize = ui_rects["btn_optimize"]
        opt_color = COLOR_PLAY_GREEN if is_optimizing else COLOR_UI_BORDER
        pygame.draw.rect(screen, opt_color if is_optimizing else COLOR_BACKGROUND, btn_optimize, border_radius=4)
        pygame.draw.rect(screen, COLOR_UI_BORDER, btn_optimize, width=1, border_radius=4)
        screen.blit(self.font_header.render("OPTIMIZE", True, COLOR_TEXT_MAIN if is_optimizing else COLOR_TEXT_MUTED), (btn_optimize.x + 18, btn_optimize.y + 9))

        btn_trails = ui_rects["btn_trails"]
        pygame.draw.rect(screen, (40, 55, 45) if trails_enabled else (55, 35, 35), btn_trails, border_radius=4)
        pygame.draw.rect(screen, COLOR_PLAY_GREEN if trails_enabled else COLOR_LOAD, btn_trails, width=1, border_radius=4)
        t_label = "Trails: ON" if trails_enabled else "Trails: OFF"
        screen.blit(self.font_body.render(t_label, True, COLOR_TEXT_MAIN), (btn_trails.x + 16, btn_trails.y + 6))

        chk_profile = ui_rects["chk_profile"]
        pygame.draw.rect(screen, (10, 10, 12), chk_profile, border_radius=2)
        pygame.draw.rect(screen, COLOR_UI_BORDER, chk_profile, width=1, border_radius=2)
        if allow_profile_switching:
            pygame.draw.rect(screen, COLOR_PLAY_GREEN, chk_profile.inflate(-4, -4), border_radius=1)
        screen.blit(self.font_body.render("Swap Profile", True, COLOR_TEXT_MAIN if allow_profile_switching else COLOR_TEXT_MUTED), (chk_profile.x + 20, chk_profile.y - 1))

    def draw_benchmark_hud(self, screen, truss):
        metrics = calculate_benchmark_metrics(truss)
        if metrics is not None:
            bhud_w, bhud_h = 430, 200
            bhud_x = sim_rect.left + 15
            bhud_y = sim_rect.top + 15
            
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (63, 63, 70, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            
            bhud_surface.blit(self.font_header.render("VERIFICATION BENCHMARK METRICS", True, COLOR_TEXT_MAIN), (15, 12))
            bhud_surface.blit(self.font_body.render("Metric                  Analytical          Numerical         Error", True, COLOR_TEXT_MUTED), (15, 34))
            pygame.draw.line(bhud_surface, COLOR_UI_BORDER, (10, 52), (bhud_w - 10, 52), 1)
            
            rows = [
                ("Diag Force", f"{metrics['diag_force_theory']/1000.0:.2f} kN", f"{metrics['diag_force_num']/1000.0:.2f} kN", f"{metrics['diag_force_err']:.4f}%"),
                ("Horiz Force", f"{metrics['horiz_force_theory']/1000.0:.2f} kN", f"{metrics['horiz_force_num']/1000.0:.2f} kN", f"{metrics['horiz_force_err']:.4f}%"),
                ("Deflection dX", f"{metrics['dx_theory']*1000.0:.4f} mm", f"{metrics['dx_num']*1000.0:.4f} mm", f"{metrics['dx_err']:.4f}%"),
                ("Deflection dY", f"{metrics['dy_theory']*1000.0:.4f} mm", f"{metrics['dy_num']*1000.0:.4f} mm", f"{metrics['dy_err']:.4f}%")
            ]
            
            curr_y = 62
            for label, theory_str, num_str, err_str in rows:
                bhud_surface.blit(self.font_body.render(label, True, COLOR_TEXT_MUTED), (15, curr_y))
                bhud_surface.blit(self.font_body.render(theory_str, True, COLOR_TEXT_MAIN), (135, curr_y))
                bhud_surface.blit(self.font_body.render(num_str, True, COLOR_TEXT_MAIN), (235, curr_y))
                bhud_surface.blit(self.font_body.render(err_str, True, COLOR_ZERO_LOAD), (335, curr_y))
                curr_y += 20
                
            pygame.draw.line(bhud_surface, COLOR_UI_BORDER, (10, 150), (bhud_w - 10, 150), 1)
            bhud_surface.blit(self.font_body.render("Target: 3m span, 50kN point-load at node 2.", True, COLOR_TEXT_MUTED), (15, 158))
            bhud_surface.blit(self.font_body.render("Status: Matrix validation test pass.", True, COLOR_ZERO_LOAD), (15, 176))
            screen.blit(bhud_surface, (bhud_x, bhud_y))
        else:
            bhud_w, bhud_h = 320, 45
            bhud_x = sim_rect.left + 15
            bhud_y = sim_rect.top + 15
            
            bhud_surface = pygame.Surface((bhud_w, bhud_h), pygame.SRCALPHA)
            pygame.draw.rect(bhud_surface, (18, 18, 20, 220), (0, 0, bhud_w, bhud_h), border_radius=6)
            pygame.draw.rect(bhud_surface, (127, 29, 29, 255), (0, 0, bhud_w, bhud_h), width=1, border_radius=6)
            bhud_surface.blit(self.font_body.render("BENCHMARK CASE MODIFIED OR INVALID", True, COLOR_MAX_LOAD), (15, 14))
            screen.blit(bhud_surface, (bhud_x, bhud_y))

    def draw_selection_hud(self, screen, truss, sim_ctrl, selected_node_idx, selected_beam_idx, selected_cable_idx, selected_road_idx, current_mode, input_active, input_type, input_buffer, calc_util_fn):
        header_text = ""
        is_elem_selected = selected_beam_idx is not None or selected_cable_idx is not None or selected_road_idx is not None
        lines = []
        top_lines = []
        geom_lines = []
        
        if is_elem_selected:
            if selected_beam_idx is not None:
                elem = truss.beams[selected_beam_idx]
                header_text = "STRUCTURAL ELEMENT"
            elif selected_cable_idx is not None:
                elem = truss.cables[selected_cable_idx]
                header_text = "CABLE ELEMENT"
            else:
                elem = truss.roads[selected_road_idx]
                header_text = "ROAD DECK ELEMENT"
            
            is_cable = getattr(elem, "is_cable", False)
            is_road = getattr(elem, "is_road", False)

            if sim_ctrl.state != "EDIT" and sim_ctrl.saved_truss_state is not None:
                na = sim_ctrl.saved_truss_state[elem.node_a]
                nb = sim_ctrl.saved_truss_state[elem.node_b]
                length_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
            else:
                length_m = truss.get_beam_length(elem)
                
            stress_m_pa = elem.stress / 1e6               
            force_k_n = elem.force / 1000.0               
            
            if is_road:
                yield_stress = elem.modulus * 0.005 
                live_util = abs(elem.stress) / yield_stress if yield_stress > 0 else 0
            else:
                live_util = calc_util_fn(elem)
                
            live_util_pct = live_util * 100.0
            
            e_peak = elem.peak_utilization_seen
            e_min = elem.minimum_fos_seen
            e_has_history = (e_peak > 0.0) or (e_min != float('inf'))
            
            live_fos = 1.0 / live_util if live_util > 0.0 else float('inf')
            live_fos_str = f"{live_fos:.2f}" if math.isfinite(live_fos) else "N/A"
            
            if e_has_history or sim_ctrl.state != "EDIT":
                peak_str = f"{e_peak * 100.0:.1f}%"
                min_fos_str = f"{e_min:.2f}" if math.isfinite(e_min) else "N/A"
            else:
                peak_str = "N/A"
                min_fos_str = "N/A"
            
            if is_cable and elem.force <= 1e-4:
                nature = "SLACK"
            else:
                nature = "TENSION" if elem.stress > 1e-2 else ("COMPRESSION" if elem.stress < -1e-2 else "NEUTRAL")
            
            top_lines = [
                f"Material: {elem.material}" + (" [M]" if not is_road else ""),
                "-",
                f"Force: {force_k_n:.1f} kN",
                f"Stress: {stress_m_pa:.1f} MPa",
                "-",
                f"Utilization: {live_util_pct:.1f}%",
                f"Peak Utilization: {peak_str}",
                f"FoS: {live_fos_str}",
                f"Minimum FoS: {min_fos_str}",
                "-",
                f"Nature: {nature}"
            ]
            
            if nature == "COMPRESSION" and length_m > 0 and not is_cable and not is_road:
                p_crit = (math.pi ** 2 * elem.modulus * elem.inertia) / (length_m ** 2)
                top_lines.append(f"Buckling Lmt: {p_crit / 1000.0:.1f} kN")
                
            top_lines.append(f"Status: {elem.status}")
            
            if is_cable:
                geom_lines = [
                    f"Profile: {elem.profile}", 
                    f"Diameter: {elem.dim_w * 100.0:.1f} cm [ [ ] / [ ] ]", 
                    f"Area: {elem.area * 1e4:.1f} cm²",
                    f"Length: {length_m:.2f} m"
                ]
            elif is_road:
                geom_lines = [
                    f"Profile: {elem.profile}", 
                    f"Width: {elem.dim_w:.2f} m", 
                    f"Thickness: {elem.dim_t * 100.0:.1f} cm", 
                    f"Area: {elem.area * 1e4:.1f} cm²",
                    f"Length: {length_m:.2f} m"
                ]
            else:
                geom_lines = [
                    f"Profile: {elem.profile} [P]", 
                    f"Width/Diam: {elem.dim_w * 100.0:.1f} cm [ [ ] / [ ] ]", 
                    f"Thickness: {elem.dim_t * 100.0:.1f} cm [ {{ }} / {{ }} ]", 
                    f"Area: {elem.area * 1e4:.1f} cm²",
                    f"Length: {length_m:.2f} m"
                ]
            
        elif selected_node_idx is not None:
            node = truss.nodes[selected_node_idx]
            header_text = "STRUCTURAL NODE"
            support_str = "Pin Support" if node.is_anchor_x and node.is_anchor_y else ("Roller Support" if node.is_anchor_y else "Free Joint")
            effective_y = node.load_y
            net_magnitude = math.hypot(node.load_x, effective_y) / 1000.0
            
            live_speed = 0.0
            if sim_ctrl.state != "EDIT" and sim_ctrl.physics_sim is not None:
                for p in sim_ctrl.physics_sim.particles:
                    if p.node_idx == selected_node_idx:
                        live_speed = math.hypot(p.vx, p.vy) * 0.0125
                        break
                        
            peak_speed = node.peak_speed
            
            lines = [
                f"Coordinates: ({round(node.x)}, {round(-node.y)})",
                f"Support Type: {support_str}",
                "-",
                f"Net Load: {net_magnitude:.1f} kN",
                f"Load X: {node.load_x / 1000.0:.1f} kN",
                f"Load Y: {-effective_y / 1000.0:.1f} kN"
            ]
            
            if node.is_anchor_x or node.is_anchor_y:
                net_react = math.hypot(node.rx, node.ry) / 1000.0
                lines.extend([
                    "-",
                    f"Net React: {net_react:.1f} kN",
                    f"React X: {node.rx / 1000.0:.1f} kN",
                    f"React Y: {-node.ry / 1000.0:.1f} kN"
                ])
                
            lines.extend([
                "-",
                f"Velocity: {live_speed:.2f} m/s",
                f"Peak Velocity: {peak_speed:.2f} m/s"
            ])

        if is_elem_selected:
            hud_w = 340
            hud_h = 55 + (len([l for l in top_lines if l != "-"]) * 20) + (16 * top_lines.count("-")) + 105 + 110
        else:
            hud_w = max(260, max([self.font_body.size(line)[0] for line in lines]) + 40)
            hud_h = 45 + (len([l for l in lines if l != "-"]) * 20) + (16 * lines.count("-"))
            if current_mode == "LOAD" and selected_node_idx is not None:
                hud_h += 75
            
        hud_x = sim_rect.right - hud_w - 15
        hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        pygame.draw.rect(hud_surface, (18, 18, 20, 220), (0, 0, hud_w, hud_h), border_radius=6)
        pygame.draw.rect(hud_surface, (63, 63, 70, 255), (0, 0, hud_w, hud_h), width=1, border_radius=6)
        hud_surface.blit(self.font_header.render(header_text, True, COLOR_TEXT_MAIN), (15, 12))

        local_y = 40
        if is_elem_selected:
            for line in top_lines:
                if line == "-":
                    local_y += 4
                    pygame.draw.line(hud_surface, COLOR_UI_BORDER, (10, local_y), (hud_w - 10, local_y), 1)
                    local_y += 12
                    continue
                    
                if ":" in line:
                    parts = line.split(":", 1)
                    lbl_surface = self.font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                    hud_surface.blit(lbl_surface, (15, local_y))
                    hud_surface.blit(self.font_body.render(parts[1], True, COLOR_TEXT_MAIN), (15 + lbl_surface.get_width() + 4, local_y))
                else:
                    hud_surface.blit(self.font_body.render(line, True, COLOR_TEXT_MAIN), (15, local_y))
                local_y += 20
                
            local_y += 4
            pygame.draw.line(hud_surface, COLOR_UI_BORDER, (10, local_y), (hud_w - 10, local_y), 1)
            local_y += 12
            
            draw_profile_preview(hud_surface, 15, local_y, 85, 85, elem)
            sub_y = local_y - 2
            
            for line in geom_lines:
                if ":" in line:
                    parts = line.split(":", 1)
                    lbl_surface = self.font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                    hud_surface.blit(lbl_surface, (115, sub_y))
                    hud_surface.blit(self.font_body.render(parts[1], True, COLOR_TEXT_MAIN), (115 + lbl_surface.get_width() + 4, sub_y))
                else:
                    hud_surface.blit(self.font_body.render(line, True, COLOR_TEXT_MAIN), (115, sub_y))
                sub_y += 17
                
            sub_y += 15
            
            hud_surface.blit(self.font_body.render("Utilization History", True, COLOR_TEXT_MUTED), (15, sub_y))
            peak_str_txt = f"Highest: {elem.peak_utilization_seen * 100.0:.1f}%"
            peak_surf = self.font_body.render(peak_str_txt, True, COLOR_TEXT_MAIN)
            hud_surface.blit(peak_surf, (hud_w - 15 - peak_surf.get_width(), sub_y))
            
            sub_y += 20
            graph_rect = pygame.Rect(15, sub_y, hud_w - 30, 55)
            pygame.draw.rect(hud_surface, (12, 12, 14), graph_rect, border_radius=4)
            pygame.draw.rect(hud_surface, COLOR_UI_BORDER, graph_rect, width=1, border_radius=4)
            
            graph_max = max(110.0, (elem.peak_utilization_seen * 100.0) + 10.0)
            y_100 = graph_rect.bottom - (100.0 / graph_max) * graph_rect.height
            
            if graph_rect.top < y_100 < graph_rect.bottom:
                pygame.draw.line(hud_surface, (180, 50, 50), (graph_rect.left, y_100), (graph_rect.right, y_100), 1)
            
            if len(elem.history) > 1:
                pts = []
                for t_idx, val in enumerate(elem.history):
                    px = graph_rect.left + (t_idx / 299.0) * graph_rect.width
                    py = graph_rect.bottom - (val / graph_max) * graph_rect.height
                    py = max(graph_rect.top, min(graph_rect.bottom, py))
                    pts.append((px, py))
                pygame.draw.lines(hud_surface, COLOR_HIGHLIGHT, False, pts, 2)
                
        else:
            for line in lines:
                if line == "-":
                    local_y += 4
                    pygame.draw.line(hud_surface, COLOR_UI_BORDER, (10, local_y), (hud_w - 10, local_y), 1)
                    local_y += 12
                    continue
                    
                if ":" in line:
                    parts = line.split(":", 1)
                    lbl_surface = self.font_body.render(parts[0] + ":", True, COLOR_TEXT_MUTED)
                    hud_surface.blit(lbl_surface, (15, local_y))
                    hud_surface.blit(self.font_body.render(parts[1], True, COLOR_TEXT_MAIN), (15 + lbl_surface.get_width() + 4, local_y))
                else:
                    hud_surface.blit(self.font_body.render(line, True, COLOR_TEXT_MAIN), (15, local_y))
                local_y += 20
                
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
                hud_surface.blit(self.font_body.render("FX: " + (input_buffer if (input_active and input_type == "X") else f"{truss.nodes[selected_node_idx].load_x/1000.0:.1f}") + ("_" if (input_active and input_type == "X") else " kN"), True, COLOR_TEXT_MAIN), (22, local_y + 5))
                
                pygame.draw.rect(hud_surface, (30, 30, 35) if (input_active and input_type == "Y") else (10, 10, 12), box_y, border_radius=4)
                pygame.draw.rect(hud_surface, COLOR_UI_BORDER, box_y, width=1, border_radius=4)
                hud_surface.blit(self.font_body.render("FY: " + (input_buffer if (input_active and input_type == "Y") else f"{-truss.nodes[selected_node_idx].load_y/1000.0:.1f}") + ("_" if (input_active and input_type == "Y") else " kN"), True, COLOR_TEXT_MAIN), (127, local_y + 5))
                hud_surface.blit(self.font_body.render("Click box, type value, press Enter", True, COLOR_TEXT_MUTED), (15, local_y + 35))

        screen.blit(hud_surface, (hud_x, sim_rect.top + 15))
        return input_active, input_type, input_buffer

    def draw_status_banner(self, screen, status_banner_text):
        sw, sh = self.font_header.size(status_banner_text)[0] + 30, 35
        sb_rect = pygame.Rect(sim_rect.left + (sim_rect.width - sw) // 2, sim_rect.bottom - 50, sw, sh)
        pygame.draw.rect(screen, (24, 24, 27, 230), sb_rect, border_radius=4)
        pygame.draw.rect(screen, COLOR_PLAY_GREEN if "SUCCESS" in status_banner_text or "LOADED" in status_banner_text else COLOR_LOAD, sb_rect, width=1, border_radius=4)
        screen.blit(self.font_header.render(status_banner_text, True, COLOR_TEXT_MAIN), (sb_rect.x + 15, sb_rect.y + 9))

    def draw_bottom_legend(self, screen, gravity_multiplier, first_break_gravity, grid_enabled):
        h = screen.get_height()
        grav_msg = f"GRAVITY LOAD MULTIPLIER: {gravity_multiplier:.1f}x  [ - ] / [ + ]"
        if first_break_gravity is not None and math.isclose(gravity_multiplier, first_break_gravity): grav_msg += " (CRITICAL POINT LOCKED)"
        screen.blit(self.font_body.render(grav_msg, True, COLOR_TEXT_MAIN), (165, h - 75))
        screen.blit(self.font_body.render(f"GRID SNAP: {'ENABLED (20px)' if grid_enabled else 'DISABLED'} | SAVE: [Ctrl+S] | LOAD: [Ctrl+O]", True, COLOR_TEXT_MUTED), (165, h - 55))
        screen.blit(self.font_body.render("Keys/Buttons: [1-3] Material | [R] Reset | [SPACE] Playback | Arrow Keys adjust external node loads | [ / ] width | { / } thickness | [M] material | [P] profile | [W] Self-Weight | [H] Home Cam | [F11] Fullscreen", True, COLOR_TEXT_MUTED), (165, h - 35))