import pygame
import math
from constants import *
from materials import MaterialManager

def find_proxy_limit(beam, truss):
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

def get_stress_color(elem, truss):
    if elem.status == "FRACTURED":
        return (180, 180, 185)
        
    is_cable = getattr(elem, "is_cable", False)
    if is_cable and elem.force <= 1e-4:
        return COLOR_CABLE
        
    if elem.force == 0.0:
        return (180, 180, 185)
        
    if elem.status == "YIELDING":
        limit, proxy_node = find_proxy_limit(elem, truss)
        if proxy_node is not None:
            yield_stress = MaterialManager.get_yield_stress(elem.material)
            util = abs(elem.stress) / yield_stress
            if min(35.0, (util - 0.95) * 18.0) >= limit:
                return (220, 38, 38)
        pulse = (math.sin(pygame.time.get_ticks() * 0.01) + 1.0) / 2.0
        return (int(249 - pulse * 15), int(115 + pulse * 25), int(22 - pulse * 10))
        
    yield_stress = MaterialManager.get_yield_stress(elem.material)
    yield_util = abs(elem.stress) / yield_stress
    is_compression = elem.stress < -1e-2
    
    if is_compression and not is_cable:
        length_m = truss.get_beam_length(elem)
        buckle_util = 0.0
        if length_m > 0:
            p_crit = MaterialManager.calculate_buckling_load(elem.material, elem.inertia, length_m)
            buckle_util = abs(elem.force) / p_crit
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
        if is_cable:
            t = utilization
            return (max(0, min(255, int(156 + (234 - 156) * t))), max(0, min(255, int(163 + (179 - 163) * t))), max(0, min(255, int(175 - (175 - 8) * t))))
        t = utilization
        return (max(0, min(255, int(180 - (180 - 34) * t))), max(0, min(255, int(180 + (211 - 180) * t))), max(0, min(255, int(185 + (238 - 185) * t))))

def draw_force_vector(surface, cx, cy, fx, fy, zoom_scale, color=COLOR_LOAD):
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

def draw_curved_beam(surface, ax, ay, bx, by, beam, thickness, color, is_selected, zoom_scale, truss):
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
    max_bow = min(35.0, (util - 0.95) * 18.0) * zoom_scale
    if max_bow < 1.0:
        max_bow = 1.0
    if beam.stress > 0.0:
        max_bow *= 0.15
    limit, proxy_node = find_proxy_limit(beam, truss)
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

def draw_cable_element(surface, ax, ay, bx, by, cable, thickness, color, is_selected, zoom_scale, truss, sim_ctrl):
    dx = bx - ax
    dy = by - ay
    L_pixels = math.hypot(dx, dy)
    if L_pixels < 1: return
    
    is_slack = cable.force <= 1e-4
    
    if is_slack and cable.status != "FRACTURED":
        L_curr_m = (L_pixels / zoom_scale) * 0.0125
        
        if sim_ctrl.state != "EDIT" and sim_ctrl.saved_truss_state is not None:
            na = sim_ctrl.saved_truss_state[cable.node_a]
            nb = sim_ctrl.saved_truss_state[cable.node_b]
            L_rest_m = math.hypot(nb["x"] - na["x"], nb["y"] - na["y"]) * 0.0125
        else:
            L_rest_m = truss.get_beam_length(cable)
            
        if L_curr_m < L_rest_m - 1e-5:
            diff = L_rest_m - L_curr_m
            droop_m = math.sqrt((0.375 * L_curr_m * diff) + (0.25 * diff * diff))
            droop_px = (droop_m / 0.0125) * zoom_scale
        else:
            droop_px = 1.0 * zoom_scale 
            
        ctrl_x = (ax + bx) / 2.0
        ctrl_y = ((ay + by) / 2.0) + (droop_px * 2.0)
        
        pts = []
        for t in range(13):
            tt = t / 12.0
            u = 1.0 - tt
            x = u*u*ax + 2*u*tt*ctrl_x + tt*tt*bx
            y = u*u*ay + 2*u*tt*ctrl_y + tt*tt*by
            pts.append((x, y))
        
        if is_selected:
            pygame.draw.lines(surface, COLOR_HIGHLIGHT, False, pts, thickness + 4)
        pygame.draw.lines(surface, color, False, pts, thickness)
    else:
        if is_selected:
            pygame.draw.line(surface, COLOR_HIGHLIGHT, (ax, ay), (bx, by), thickness + 4)
        pygame.draw.line(surface, color, (ax, ay), (bx, by), thickness)