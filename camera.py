import pygame
from constants import sim_rect, WORKSPACE_LIMIT

class Camera:
    def __init__(self):
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.zoom_scale = 1.0
        self.is_panning = False
        self.last_mouse_pos = (0, 0)
        self.MIN_ZOOM = 0.3
        self.MAX_ZOOM = 4.0
        self.WORKSPACE_LIMIT = WORKSPACE_LIMIT

    def reset(self):
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.zoom_scale = 1.0
        self.last_mouse_pos = (0, 0)

    def to_screen(self, sim_x, sim_y):
        cx = sim_rect.left + sim_rect.width / 2
        cy = sim_rect.top + sim_rect.height / 2
        scr_x = cx + (sim_x - cx) * self.zoom_scale + self.pan_x
        scr_y = cy + (sim_y - cy) * self.zoom_scale + self.pan_y
        return scr_x, scr_y

    def to_sim(self, screen_x, screen_y):
        cx = sim_rect.left + sim_rect.width / 2
        cy = sim_rect.top + sim_rect.height / 2
        sim_x = (screen_x - self.pan_x - cx) / self.zoom_scale + cx
        sim_y = (screen_y - self.pan_y - cy) / self.zoom_scale + cy
        return sim_x, sim_y

    def start_panning(self, mouse_pos):
        self.is_panning = True
        self.last_mouse_pos = mouse_pos

    def update_pan(self, mouse_pos):
        if self.is_panning:
            dx = mouse_pos[0] - self.last_mouse_pos[0]
            dy = mouse_pos[1] - self.last_mouse_pos[1]
            self.pan_x = max(-self.WORKSPACE_LIMIT, min(self.WORKSPACE_LIMIT, self.pan_x + dx))
            self.pan_y = max(-self.WORKSPACE_LIMIT, min(self.WORKSPACE_LIMIT, self.pan_y + dy))
            self.last_mouse_pos = mouse_pos

    def update(self, mouse_pos):
        self.update_pan(mouse_pos)

    def stop_panning(self):
        self.is_panning = False

    def handle_zoom(self, mouse_pos, direction):
        cx = sim_rect.left + sim_rect.width / 2
        cy = sim_rect.top + sim_rect.height / 2
        
        sim_x = cx + (mouse_pos[0] - cx - self.pan_x) / self.zoom_scale
        sim_y = cy + (mouse_pos[1] - cy - self.pan_y) / self.zoom_scale
        
        if direction == "in":
            self.zoom_scale = min(self.MAX_ZOOM, self.zoom_scale * 1.1)
        elif direction == "out":
            self.zoom_scale = max(self.MIN_ZOOM, self.zoom_scale / 1.1)
            
        self.pan_x = mouse_pos[0] - cx - (sim_x - cx) * self.zoom_scale
        self.pan_y = mouse_pos[1] - cy - (sim_y - cy) * self.zoom_scale

    def process_event(self, event, mouse_pos):
        if event.type == pygame.MOUSEWHEEL and sim_rect.collidepoint(mouse_pos):
            if event.y > 0:
                self.handle_zoom(mouse_pos, "in")
            elif event.y < 0:
                self.handle_zoom(mouse_pos, "out")
            return True

        if event.type == pygame.MOUSEBUTTONDOWN:
            if sim_rect.collidepoint(event.pos) and (event.button == 2 or (event.button == 1 and pygame.key.get_pressed()[pygame.K_LSHIFT])):
                self.start_panning(event.pos)
                return True

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button in (1, 2) and self.is_panning:
                self.stop_panning()
                return True

        return False