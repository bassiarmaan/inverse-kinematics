import numpy as np
import pygame

l1 = 1  #length of the first segment
l2 = 1  #length of the second segment
B = np.array([0.0, 0.0])  #base position



def calculate_angles(target, l1, l2):
    x, y = target
    distance = np.sqrt(x ** 2 + y ** 2)

    #check if target is reachable
    if distance > l1 + l2:  #target too far
        scale = (l1 + l2) / distance
        x *= scale
        y *= scale
    elif distance < abs(l1 - l2):  #target too close
        scale = abs(l1 - l2) / distance
        x *= scale
        y *= scale

    cos_theta1 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cos_theta1 = np.clip(cos_theta1, -1, 1)
    theta1 = np.arccos(cos_theta1)

    beta = np.arctan2(y, x)
    cos_alpha = (x ** 2 + y ** 2 + l1 ** 2 - l2 ** 2) / (2 * l1 * np.sqrt(x ** 2 + y ** 2))
    cos_alpha = np.clip(cos_alpha, -1, 1)
    alpha = np.arccos(cos_alpha)
    theta0 = beta - alpha

    return theta0, theta0 + theta1


#calculate joint and end-effector positions
def calculate_positions(B, theta0, theta1, l1, l2):
    S1 = B + np.array([np.cos(theta0), np.sin(theta0)]) * l1
    S2 = S1 + np.array([np.cos(theta1), np.sin(theta1)]) * l2
    return S1, S2

#initialize Pygame
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

#screen coordinate conversion functions
def to_screen(coord):
    return int(width / 2 + coord[0] * 100), int(height / 2 - coord[1] * 100)

def from_screen(coord):
    return np.array([(coord[0] - width / 2) / 100, -(coord[1] - height / 2) / 100])


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #get mouse position
    mouse_pos = pygame.mouse.get_pos()
    target = from_screen(mouse_pos)

    #make background white
    screen.fill((255, 255, 255))

    try:
        #calculate angles and positions
        theta0, theta1 = calculate_angles(target, l1, l2)
        S1, S2 = calculate_positions(B, theta0, theta1, l1, l2)

        #draw arm segments
        pygame.draw.line(screen, (0, 0, 0), to_screen(B), to_screen(S1), 3)
        pygame.draw.line(screen, (0, 0, 0), to_screen(S1), to_screen(S2), 3)

        #draw joints and target
        pygame.draw.circle(screen, (0, 0, 255), to_screen(B), 5)
        pygame.draw.circle(screen, (0, 255, 0), to_screen(S1), 5)
        pygame.draw.circle(screen, (255, 0, 0), to_screen(S2), 5)
        pygame.draw.circle(screen, (128, 0, 128), mouse_pos, 5)

    except:
        pass #handle any numerical errors easily

    pygame.display.flip()
    clock.tick(60)

pygame.quit()