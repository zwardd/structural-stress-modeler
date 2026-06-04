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
        scr_x = cx + sim_x * self.zoom_scale + self.pan_x
        scr_y = cy + sim_y * self.zoom_scale + self.pan_y
        return scr_x, scr_y

    def to_sim(self, screen_x, screen_y):
        cx = sim_rect.left + sim_rect.width / 2
        cy = sim_rect.top + sim_rect.height / 2
        sim_x = (screen_x - cx - self.pan_x) / self.zoom_scale
        sim_y = (screen_y - cy - self.pan_y) / self.zoom_scale
        return sim_x, sim_y

    def update(self, mouse_pos):
        mods = pygame.key.get_mods()
        is_middle_click = pygame.mouse.get_pressed()[1]
        is_shift_left_click = pygame.mouse.get_pressed()[0] and (mods & pygame.KMOD_SHIFT)
        
        if is_middle_click or is_shift_left_click:
            if not self.is_panning:
                self.is_panning = True
                self.last_mouse_pos = mouse_pos
            else:
                dx = mouse_pos[0] - self.last_mouse_pos[0]
                dy = mouse_pos[1] - self.last_mouse_pos[1]
                self.pan_x += dx
                self.pan_y += dy
                self.last_mouse_pos = mouse_pos
        else:
            self.is_panning = False

    def handle_zoom(self, mouse_pos, direction):
        sim_x, sim_y = self.to_sim(mouse_pos[0], mouse_pos[1])
        
        if direction == "in":
            self.zoom_scale = min(self.MAX_ZOOM, self.zoom_scale * 1.1)
        elif direction == "out":
            self.zoom_scale = max(self.MIN_ZOOM, self.zoom_scale / 1.1)
            
        cx = sim_rect.left + sim_rect.width / 2
        cy = sim_rect.top + sim_rect.height / 2
        self.pan_x = mouse_pos[0] - cx - sim_x * self.zoom_scale
        self.pan_y = mouse_pos[1] - cy - sim_y * self.zoom_scale

    def process_event(self, event, mouse_pos):
        if event.type == pygame.MOUSEWHEEL and sim_rect.collidepoint(mouse_pos):
            if event.y > 0:
                self.handle_zoom(mouse_pos, "in")
            elif event.y < 0:
                self.handle_zoom(mouse_pos, "out")
            return True
        return False