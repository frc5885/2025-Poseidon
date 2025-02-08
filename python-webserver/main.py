#!/usr/bin/env python3
import pygame
import math
import sys
import time
from networktables import NetworkTables
import logging


# Initialize Pygame
pygame.init()

# Initialize NetworkTables
logging.basicConfig(level=logging.DEBUG)
NetworkTables.initialize(server='127.0.0.1')  # Replace with robot IP
sd = NetworkTables.getTable("ReefPanel")


# Configuration
WINDOW_SIZE = (1024, 600)
HEX_RADIUS = 270  # Radius of the hexagon
BUTTON_LENGTH = 100  # Length of rectangular buttons
BUTTON_WIDTH = 40   # Width of rectangular buttons
ROTATION_ANGLE = 90  # Rotate hexagon by 90 degrees
COLORS = {
    'background': (255, 255, 255),
    'hexagon': (0, 0, 0),
    'button_default': (0, 120, 215),
    'button_active': (255, 165, 0)
}

# Create display window
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("Reef Panel")
clock = pygame.time.Clock()

class HexagonButton:
    def __init__(self, center, angle, width, length):
        """Initialize a hexagonal button
        Args:
            center (tuple): (x, y) position of button center
            angle (float): Rotation angle in radians
        """
        self.width = width
        self.length = length
        self.center = center
        self.angle = angle  # Store angle in radians
        self.rect = pygame.Rect(0, 0, length, width)
        self.rect.center = center
        self.is_active = False

    def draw(self, surface):
        """Draw rotated button on specified surface"""
        # Create temporary surface for rotation
        temp_surface = pygame.Surface((BUTTON_LENGTH, BUTTON_WIDTH), pygame.SRCALPHA)
        temp_surface.fill(COLORS['button_active' if self.is_active else 'button_default'])
        
        # Rotate and position
        rotated_surf = pygame.transform.rotate(temp_surface, math.degrees(-self.angle))
        new_rect = rotated_surf.get_rect(center=self.rect.center)
        
        surface.blit(rotated_surf, new_rect.topleft)

    def check_click(self, mouse_pos):
        """Check if mouse position is within button bounds"""
        # Convert to vector for rotation calculation
        vec = pygame.math.Vector2(mouse_pos) - self.center
        rotated_vec = vec.rotate(-math.degrees(self.angle))
        
        # Check boundaries
        return (abs(rotated_vec.x) < BUTTON_LENGTH/2 and 
                abs(rotated_vec.y) < BUTTON_WIDTH/2)

def calculate_hexagon_points(center, radius, rotation=0):
    """Generate hexagon vertex coordinates with rotation
    Args:
        center (tuple): (x, y) of hexagon center
        radius (float): Hexagon radius
        rotation (float): Rotation angle in degrees
    Returns:
        list: Six (x, y) tuples representing vertices
    """
    return [(center[0] + radius * math.cos(math.radians(angle + rotation)),
             center[1] + radius * math.sin(math.radians(angle + rotation)))
            for angle in range(30, 390, 60)]

def create_buttons(center, radius, rotation=0):
    """Generate buttons along hexagon edges with rotation
    Args:
        center (tuple): (x, y) of hexagon center
        radius (float): Hexagon radius
        rotation (float): Rotation angle in degrees
    Returns:
        list: HexagonButton objects
    """
    buttons = []
    vertices = calculate_hexagon_points(center, radius, rotation)
    
    # Process each edge
    for i in range(6):
        start = vertices[i]
        end = vertices[(i+1)%6]
        
        # Calculate edge vector
        edge_vector = (end[0]-start[0], end[1]-start[1])
        edge_angle = math.atan2(edge_vector[1], edge_vector[0])
        
        # Create three buttons per edge
        for t in [9/30, 21/30]:
            # Calculate button position
            x = start[0] + t * edge_vector[0]
            y = start[1] + t * edge_vector[1]
            
            buttons.append(HexagonButton((x, y), edge_angle,BUTTON_WIDTH, BUTTON_LENGTH))
    
    return buttons

def creatLevelButtons():
    buttons = []
    for i in range(1,5):
        buttons.append(HexagonButton((850, 20 + 110*i), 0,200, 100 ))
    return buttons
def main():
    # Calculate initial positions with rotation
    center = (WINDOW_SIZE[0]//2-180, WINDOW_SIZE[1]//2)
    buttons = create_buttons(center, HEX_RADIUS, ROTATION_ANGLE)
    active_button = None  # Track the currently active button

    levelbuttons = creatLevelButtons()
    active_buttonLevel = levelbuttons[0]
    
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                for btnl in levelbuttons:
                    if btnl.check_click(mouse_pos):
                        if active_buttonLevel:
                            active_buttonLevel.is_active = False
                        btnl.is_active = True  # Activate new button
                        active_buttonLevel = btnl  # Update active button
                for btn in buttons:
                    if btn.check_click(mouse_pos):
                        if active_button:
                            active_button.is_active = False  # Deactivate previous button
                            active_buttonLevel.is_active = False
                        btn.is_active = True  # Activate new button
                        active_button = btn  # Update active button

                for btn in buttons:
                    if btn.is_active:
                        sd.putNumber("ReefTargets", buttons.index(btn))
                        sd.putNumber("ReefTargetsLevel",0)
                for btnl in levelbuttons:
                    if btnl.is_active:
                        sd.putNumber("ReefTargetsLevel", 4-levelbuttons.index(btnl))
  
        # Draw elements
        screen.fill(COLORS['background'])
        
        # Draw hexagon outline with rotation
        pygame.draw.polygon(screen, COLORS['hexagon'], 
                           calculate_hexagon_points(center, HEX_RADIUS, ROTATION_ANGLE), 2)
        
        # Draw buttons
        for btn in buttons:
            btn.draw(screen)
        for butn in levelbuttons:
            butn.draw(screen)
        
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()