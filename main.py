import pygame
import sys

pygame.init()

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Structural Stress Modeler - 2D Truss Analysis")

# Color Palette
COLOR_BACKGROUND = (24, 24, 27)      
COLOR_SIM_ZONE   = (33, 33, 38)      
COLOR_PANEL_BG   = (18, 18, 20)      
COLOR_UI_BORDER  = (63, 63, 70)      
COLOR_TEXT_MAIN  = (244, 244, 245)    

font_header = pygame.font.SysFont("Helvetica", 24, bold=True)
font_body = pygame.font.SysFont("Helvetica", 16)

clock = pygame.time.Clock()
is_running = True

while is_running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False

    screen.fill(COLOR_BACKGROUND)

    # Render Simulation Canvas
    sim_rect = pygame.Rect(20, 20, 680, 660)
    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    # Render Side UI Panel
    panel_rect = pygame.Rect(720, 20, 260, 660)
    pygame.draw.rect(screen, COLOR_PANEL_BG, panel_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, panel_rect, width=2, border_radius=8)

    # UI Text Display
    header_surface = font_header.render("SYSTEM METRICS", True, COLOR_TEXT_MAIN)
    screen.blit(header_surface, (740, 40))

    metrics = [
        "Total Nodes: 0",
        "Structural Beams: 0",
        "Applied Load: 0.00 kN",
        "Max Material Stress: 0.0%",
        "Safety Status: NOMINAL"
    ]

    text_y_position = 90
    for text_line in metrics:
        text_surface = font_body.render(text_line, True, COLOR_TEXT_MAIN)
        screen.blit(text_surface, (740, text_y_position))
        text_y_position += 30

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()