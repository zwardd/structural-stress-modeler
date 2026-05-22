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
COLOR_NODE       = (59, 130, 246)   
COLOR_BEAM       = (113, 113, 122)

font_header = pygame.font.SysFont("Helvetica", 24, bold=True)
font_body = pygame.font.SysFont("Helvetica", 16)

clock = pygame.time.Clock()

# Structural Data Arrays
nodes = []  
beams = []  

selected_node = None
NODE_RADIUS = 6
CLICK_TOLERANCE = 12

is_running = True
while is_running:
    
    sim_rect = pygame.Rect(20, 20, 680, 660)
    panel_rect = pygame.Rect(720, 20, 260, 660)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            is_running = False
            
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  
                mouse_pos = event.pos
                
                if sim_rect.collidepoint(mouse_pos):
                    # Check if clicking an existing node
                    clicked_node_idx = None
                    for i, node in enumerate(nodes):
                        distance = pygame.math.Vector2(node).distance_to(mouse_pos)
                        if distance < CLICK_TOLERANCE:
                            clicked_node_idx = i
                            break
                    
                    if clicked_node_idx is not None:
                        if selected_node is None:
                            selected_node = clicked_node_idx
                        else:
                            # Connect nodes if a different one is selected
                            if selected_node != clicked_node_idx:
                                new_beam = (selected_node, clicked_node_idx)
                                # Avoid duplicate tracking
                                reverse_beam = (clicked_node_idx, selected_node)
                                if new_beam not in beams and reverse_beam not in beams:
                                    beams.append(new_beam)
                            selected_node = None # Deselect
                    else:
                        # Drop a new node if clicking empty space
                        nodes.append(mouse_pos)
                        selected_node = None

    screen.fill(COLOR_BACKGROUND)

    # Render Simulation Canvas
    pygame.draw.rect(screen, COLOR_SIM_ZONE, sim_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, sim_rect, width=2, border_radius=8)

    # Render Active Beams
    for beam in beams:
        start_pos = nodes[beam[0]]
        end_pos = nodes[beam[1]]
        pygame.draw.line(screen, COLOR_BEAM, start_pos, end_pos, 3)

    # Render Active Nodes
    for i, node in enumerate(nodes):
        # Highlight node if currently selected for a beam connection
        color = (234, 179, 8) if i == selected_node else COLOR_NODE
        pygame.draw.circle(screen, color, node, NODE_RADIUS)

    # Render Side UI Panel
    pygame.draw.rect(screen, COLOR_PANEL_BG, panel_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_UI_BORDER, panel_rect, width=2, border_radius=8)

    # UI Text Display
    header_surface = font_header.render("SYSTEM METRICS", True, COLOR_TEXT_MAIN)
    screen.blit(header_surface, (740, 40))

    metrics = [
        f"Total Nodes: {len(nodes)}",
        f"Structural Beams: {len(beams)}",
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