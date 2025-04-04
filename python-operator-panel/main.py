#!/usr/bin/env python3
import pygame
import math
import sys
import time
from networktables import NetworkTables
import logging

# Initialize Pygame
pygame.init()

# Initialize logging
logging.basicConfig(level=logging.DEBUG)

# Configuration
WINDOW_SIZE = (1024, 600)
HEX_RADIUS = 270  # Radius of the hexagon
BUTTON_LENGTH = 200  # Length of rectangular buttons
BUTTON_WIDTH = 75   # Width of rectangular buttons
ROTATION_ANGLE = 90  # Rotate hexagon by 90 degrees
BUTTON_ANGLE = 30
COLORS = {
    'background': (21, 23, 27),
    'hexagon': (0, 0, 0),
    'button_default': (0, 72, 167),
    'button_active': (255, 165, 0),
    'text': (0, 0, 0),
    'toggle_bg': (200, 200, 200),
    'toggle_fg': (0, 72, 167),
    'toggle_text': (0, 0, 0)
}

# Create display window
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("Reef Panel")
clock = pygame.time.Clock()

# Font setup
pygame.font.init()
font = pygame.font.SysFont(None, 48)
small_font = pygame.font.SysFont(None, 36)


# Reef graphics
reef = pygame.image.load("python-operator-panel/reef.png")

leftBranch = pygame.image.load("python-operator-panel/branch.png").convert_alpha()
leftBranch = pygame.transform.scale(leftBranch, (126, 402))
leftBranch = pygame.transform.flip(leftBranch, True, False)
rightBranch = pygame.image.load("python-operator-panel/branch.png").convert_alpha()
rightBranch = pygame.transform.scale(rightBranch, (126, 402))

programIcon = pygame.image.load('python-operator-panel\WiredcatsVectorLogo.png')
pygame.display.set_icon(programIcon)

class Spinner:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.angle = 0
        self.surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        
    def draw(self, surface):
        # Clear previous frame
        self.surface.fill((0, 0, 0, 0))
        
        # Draw arc segments with varying opacity
        num_segments = 8
        segment_angle = 360 / num_segments
        for i in range(num_segments):
            start_angle = self.angle + i * segment_angle
            # Calculate opacity based on position
            opacity = int(255 * (i / num_segments))
            color = (*COLORS['button_default'], opacity)
            
            # Draw arc segment
            pygame.draw.arc(self.surface, color, 
                          (4, 4, self.radius * 2 - 8, self.radius * 2 - 8),
                          math.radians(start_angle), 
                          math.radians(start_angle + segment_angle - 2), 
                          4)
        
        # Update rotation
        self.angle = (self.angle + 5) % 360
        
        # Draw the spinner
        rect = self.surface.get_rect(center=self.center)
        surface.blit(self.surface, rect)

class ToggleSwitch:
    def __init__(self, x, y, width=200, height=40):
        self.rect = pygame.Rect(x, y, width, height)
        self.is_on = True
        self.animation_progress = 1.0  # 0.0 to 1.0
        self.target_progress = 1.0
        
    def draw(self, surface):
        # Draw background
        pygame.draw.rect(surface, COLORS['toggle_bg'], self.rect, border_radius=self.rect.height//2)
        
        # Calculate knob position
        knob_radius = self.rect.height // 2 - 4
        knob_x = self.rect.x + self.rect.height//2 + (self.rect.width - self.rect.height) * self.animation_progress
        knob_y = self.rect.centery
        
        # Draw labels
        robot_text = small_font.render("Robot", True, COLORS['toggle_text'])
        sim_text = small_font.render("Sim", True, COLORS['toggle_text'])
        
        # Position labels
        robot_pos = (self.rect.x + 20, self.rect.centery - robot_text.get_height()//2)
        sim_pos = (self.rect.right - sim_text.get_width() - 20, self.rect.centery - sim_text.get_height()//2)
        
        # Draw both labels with appropriate opacity
        robot_opacity = int(255 * self.animation_progress)
        sim_opacity = int(255 * (1 - self.animation_progress))
        
        robot_surface = pygame.Surface(robot_text.get_size(), pygame.SRCALPHA)
        robot_surface.blit(robot_text, (0, 0))
        robot_surface.set_alpha(robot_opacity)
        surface.blit(robot_surface, robot_pos)
        
        sim_surface = pygame.Surface(sim_text.get_size(), pygame.SRCALPHA)
        sim_surface.blit(sim_text, (0, 0))
        sim_surface.set_alpha(sim_opacity)
        surface.blit(sim_surface, sim_pos)
        
        # Draw knob with a subtle shadow
        shadow_radius = knob_radius + 2
        pygame.draw.circle(surface, (0, 0, 0, 64), (knob_x, knob_y + 2), shadow_radius)
        pygame.draw.circle(surface, COLORS['toggle_fg'], (knob_x, knob_y), knob_radius)
        
        # Add a subtle highlight to the knob
        highlight_pos = (knob_x - knob_radius//3, knob_y - knob_radius//3)
        highlight_radius = knob_radius//3
        pygame.draw.circle(surface, (200, 200, 200, 128), highlight_pos, highlight_radius)
        
    def update(self):
        # Animate the toggle
        if self.animation_progress < self.target_progress:
            self.animation_progress = min(1.0, self.animation_progress + 0.15)
        elif self.animation_progress > self.target_progress:
            self.animation_progress = max(0.0, self.animation_progress - 0.15)
    
    def toggle(self):
        self.is_on = not self.is_on
        self.target_progress = 1.0 if self.is_on else 0.0
        
    def check_click(self, pos):
        return self.rect.collidepoint(pos)

class ConnectionManager:
    def __init__(self):
        self.is_robot_mode = True
        self.robot_ip = "10.58.85.2"
        self.sim_ip = "127.0.0.1"
        self.initialize_networktables()
        
    def initialize_networktables(self):
        ip = self.robot_ip if self.is_robot_mode else self.sim_ip
        NetworkTables.initialize(server=ip)
        self.sd = NetworkTables.getTable("ReefPanel")
        
    def toggle_mode(self):
        self.is_robot_mode = not self.is_robot_mode
        self.initialize_networktables()
        
    def get_current_mode(self):
        return "Robot" if self.is_robot_mode else "Simulation"

class ConnectingScreen:
    def __init__(self):
        self.spinner = Spinner((WINDOW_SIZE[0]//2, WINDOW_SIZE[1]//2), 40)
        self.toggle = ToggleSwitch(WINDOW_SIZE[0]//2 - 100, WINDOW_SIZE[1]//2 + 50)
        
    def draw(self, surface, connection_manager):
        surface.fill(COLORS['background'])
        
        # Draw "Trying to connect" text
        text = font.render("Trying to connect...", True, (200, 200, 200))
        text_rect = text.get_rect(center=(WINDOW_SIZE[0]//2, WINDOW_SIZE[1]//2 - 100))
        surface.blit(text, text_rect)
        
        # Draw spinner
        self.spinner.draw(surface)
        
        # Draw and update toggle switch
        self.toggle.update()
        self.toggle.draw(surface)

    def check_mode_button(self, pos):
        return self.toggle.check_click(pos)

# Font setup
pygame.font.init()
font = pygame.font.SysFont(None, 48)
small_font = pygame.font.SysFont(None, 36)
button_font = pygame.font.SysFont(None, 28)  # New font for button numbers

class HexagonButton:
    def __init__(self, center, angle, width, length, isRight, level):
        """Initialize button with flag for level button"""
        self.width = width
        self.length = length
        self.center = center
        self.angle = angle
        self.rect = pygame.Rect(0, 0, length, width)
        self.rect.center = center
        self.is_active = False
        self.isRight = isRight
        self.level = level

    def draw(self, surface):
        temp_surface = pygame.Surface((self.length, self.width), pygame.SRCALPHA)
        bg_color = COLORS['button_active' if self.is_active else 'button_default']
        temp_surface.fill(bg_color)
        
        # Different text handling for hexagon vs level buttons
        if self.isRight:
            display_text = "R " + str(self.level)
        else:
            display_text = "L " + str(self.level)
            
        text_color = COLORS['text'] if self.is_active else (255, 255, 255)
        text_surf = button_font.render(display_text, True, text_color)
        text_rect = text_surf.get_rect(center=(self.length//2, self.width//2))
        temp_surface.blit(text_surf, text_rect)
        
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

# def calculate_hexagon_points(center, radius, rotation=0):
#     """Generate hexagon vertex coordinates with rotation
#     Args:
#         center (tuple): (x, y) of hexagon center
#         radius (float): Hexagon radius
#         rotation (float): Rotation angle in degrees
#     Returns:
#         list: Six (x, y) tuples representing vertices
#     """
#     return [(center[0] + radius * math.cos(math.radians(angle + rotation)),
#              center[1] + radius * math.sin(math.radians(angle + rotation)))
#             for angle in range(30, 390, 60)]

# def create_buttons(center, radius, rotation=0):
#     """Generate numbered buttons along hexagon edges"""
#     buttons = []
#     button_number = 1  # Start numbering from 1
#     vertices = calculate_hexagon_points(center, radius, rotation)
    
#     for i in range(6):
#         start = vertices[i]
#         end = vertices[(i+1)%6]
#         edge_vector = (end[0]-start[0], end[1]-start[1])
#         edge_angle = math.atan2(edge_vector[1], edge_vector[0])
        
#         # Add 180 degrees (pi radians) for bottom edges
#         # Bottom edges are indices 0, 1, 5 in the hexagon
#         if i in [0, 1, 5]:
#             edge_angle += math.pi
        
#         # Create two buttons per edge with numbers
#         for t in [9/30, 21/30]:
#             x = start[0] + t * edge_vector[0]
#             y = start[1] + t * edge_vector[1]
#             buttons.append(HexagonButton((x, y), edge_angle, 
#                                       BUTTON_WIDTH, BUTTON_LENGTH,
#                                       button_number,))
#             button_number += 1  # Increment for next button
    
#     return buttons

def createButtons():
    buttons = []
    for i in range(1,5):
        button = HexagonButton((250, 20 + 110*i), 0, BUTTON_WIDTH, BUTTON_LENGTH, False, 5-i)
        buttons.append(button)
        button = HexagonButton((774, 20 + 110*i), 0, BUTTON_WIDTH, BUTTON_LENGTH, True, 5-i)
        buttons.append(button)
    return buttons



def main():
    connection_manager = ConnectionManager()
    connecting_screen = ConnectingScreen()
    
    # Calculate initial positions with rotation
    center = (WINDOW_SIZE[0]//2-180, WINDOW_SIZE[1]//2)
    # buttons = create_buttons(center, HEX_RADIUS, BUTTON_ANGLE)
    active_button = None  # Track the currently active button

    buttons = createButtons()
    # active_buttonLevel = levelButtons[0]
    
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                
                if not NetworkTables.isConnected():
                    if connecting_screen.check_mode_button(mouse_pos):
                        connecting_screen.toggle.toggle()
                        connection_manager.toggle_mode()
                else:
                    for button in buttons:
                        if button.check_click(mouse_pos):
                            if active_button:
                                active_button.is_active = False  # Deactivate previous button
                            button.is_active = True  # Activate new button
                            active_button = button  # Update active button

                    for button in buttons:
                        if button.is_active:
                            if button.isRight:
                                connection_manager.sd.putNumber("ReefTargets", 0)
                            else:
                                connection_manager.sd.putNumber("ReefTargets", 1)   
                            connection_manager.sd.putNumber("ReefTargetsLevel", button.level)
                    # for btnl in levelButtons:
                    #     if btnl.is_active:
                    #         connection_manager.sd.putNumber("ReefTargetsLevel", 4 - levelButtons.index(btnl))
        
        # Draw elements
        if not NetworkTables.isConnected():
            connecting_screen.draw(screen, connection_manager)
        else:
            screen.fill(COLORS['background'])
            # screen.blit(reef, (21, 56))
            screen.blit(leftBranch, (374, 100))
            screen.blit(rightBranch, (524, 100))
            # Draw hexagon outline with rotation
            # pygame.draw.polygon(screen, COLORS['hexagon'], 
            #                    calculate_hexagon_points(center, HEX_RADIUS, ROTATION_ANGLE), 2)
            
            # Draw buttons
            for button in buttons:
                button.draw(screen)

            # for btn in buttons:
            #     if btn.is_active:
            #         # Update level button numbers based on active hexagon button
            #         base = 4 if btn.number % 2 != 0 else 8
            #         for i, level_btn in enumerate(levelButtons):
            #             level_btn.display_number = base - i
            
            # Draw letters only
            # for i, letter in enumerate(letters):
            #     x, y = positions[i]
            #     text_surface = font.render(letter, True, (255, 255, 255))
            #     text_rect = text_surface.get_rect(center=(x, y))
            #     screen.blit(text_surface, text_rect)  # Draw text
        
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
