import pygame

WINDOW_WIDTH = 1140
WINDOW_HEIGHT = 700
GRID_SIZE = 20
CLICK_TOLERANCE = 12
NODE_RADIUS = 6
WORKSPACE_LIMIT = 50000.0

sim_rect = pygame.Rect(160, 20, 960, 660)

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
COLOR_CABLE      = (156, 163, 175)
COLOR_ROAD       = (75, 85, 99)

COLOR_ZERO_LOAD = (144, 238, 144)   
COLOR_MID_LOAD  = (234, 179, 8)     
COLOR_MAX_LOAD  = (239, 68, 68)     

MATERIAL_SPECS = {
    "Steel": {"yield": 250e6, "ultimate": 400e6, "modulus": 200e9, "density": 7850, "label": "Structural Steel"},
    "Aluminum": {"yield": 275e6, "ultimate": 310e6, "modulus": 70e9, "density": 2700, "label": "6061-T6 Aluminum"},
    "Titanium": {"yield": 880e6, "ultimate": 950e6, "modulus": 115e9, "density": 4430, "label": "Ti-6Al-4V Titanium"},
    "Steel Wire Rope": {"yield": 1200e6, "ultimate": 1400e6, "modulus": 100e9, "density": 7800, "label": "Steel Wire Rope"},
    "Polyester Rope": {"yield": 100e6, "ultimate": 120e6, "modulus": 10e9, "density": 1380, "label": "Polyester Rope"},
    "Nylon Rope": {"yield": 75e6, "ultimate": 90e6, "modulus": 3e9, "density": 1140, "label": "Nylon Rope"}
}