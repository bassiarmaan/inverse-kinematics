import numpy as np
import pygame

# length of segments
l = [1.0, 1.0, 1.0, 1.0]

# initial position
B = np.array([0.0, 0.0])


# implementing FABRIK algorithm

# target: where our end goal is
# positions: the array of the endpoints of each arm (constantly changing)
# lengths: the lengths of the arms (which can be adjusted later)
# tolerance: how close we have to be to the target for FABRIK to stop
# max_iterations: prevents algorithm from infinite looping
def fabrik(target, positions, lengths, tolerance=0.001, max_iterations=10):
    target = np.array(target)
    total_length = sum(lengths)
    # get distance from origin to target point
    distance_to_target = np.linalg.norm(target - positions[0])

    # checking if the target is too far out of reach
    if distance_to_target > total_length:
        # go through each end points of arms and stretch them all out
        for i in range(1, len(positions)):
            # distance from target to the arm before, scaling the arms toward the target
            r = np.lingalg.norm(target.positions[i - 1])
            # scaling factor that ensures distance between endpoints is the same as their lengths
            lambda_ = lengths[i - 1] / r
            # get new position of each joint and moves it towards the target, multiplied by scaling factor
            positions[i] = (1 - lambda_) * positions[i - 1] + lambda_ * target
    else:
        # the target is possible to reach
        original_base = positions[0].copy()
        diff = np.linalg.norm(positions[-1] - target)
        iterations = 0

        while diff > tolerance and iterations < max_iterations:
            # forward reaching

            # starting from end effector and moving back to the origin
            positions[-1] = target
            for i in reversed(range(len(positions) - 1)):
                # distance from current joint and the next joint
                r = np.linalg.norm(positions[i + 1] - positions[i])
                # how much to move the current joint to preserve the segment length
                lambda_ = lengths[i] / r
                # moves joints closer or farther to the next joint so the distance equals length[i]
                positions[i] = (1 - lambda_) * positions[i + 1] + lambda_ * positions[i]
            # backward reaching

            # set the base joint back to its original position
            positions[0] = original_base
            for i in range(len(positions) - 1):
                # distance from current joint and the next joint
                r = np.linalg.norm(positions[i + 1] - positions[i])
                # how much to move the next joint to preserve the segment length
                lambda_ = lengths[i] / r
                # moves the next joint closer or farther to the current joint so the distance equals lengths[i]
                positions[i + 1] = (1 - lambda_) * positions[i] + lambda_ * positions[i + 1]
            # calculate the distance between end effector and target
            diff = np.linalg.norm(positions[-1] - target)
            # increment the iteration count
            iterations += 1

    return positions

class Slider:
    def __init__(self, x, y, width, min_val, max_val, initial_val, label):
        self.x = x
        self.y = y
        self.width = width
        self.height = 10
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.label = label
        self.dragging = False
        self.handle_radius = 8

    def draw(self, screen):
        #draw slider track
        pygame.draw.rect(screen, (200, 200, 200), (self.x, self.y, self.width, self.height))

        #calculate handle position
        handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width

        #draw handle
        pygame.draw.circle(screen, (100, 100, 100), (int(handle_x), self.y + self.height // 2), self.handle_radius)

        #draw label and value
        font = pygame.font.Font(None, 24)
        label_text = font.render(f"{self.label}: {self.value:.2f}", True, (0, 0, 0))
        screen.blit(label_text, (self.x, self.y - 20))

    #dealing with the slider interaction with the mouse
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width
            handle_rect = pygame.Rect(handle_x - self.handle_radius, self.y - self.handle_radius, self.handle_radius * 2, self.handle_radius * 2 + self.height)
            if handle_rect.collidepoint(mouse_pos):
                self.dragging = True

        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False

        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mouse_x = event.pos[0]
            relative_x = max(0, min(mouse_x - self.x, self.width))
            self.value = self.min_val + (relative_x / self.width) * (self.max_val - self.min_val)

#initialize Pygame
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

#create sliders for each segment length
slider1 = Slider(50, 50, 200, 0.1, 3.0, l[0], "Arm 1 Length")
slider2 = Slider(50, 100, 200, 0.1, 3.0, l[1], "Arm 2 Length")
slider3 = Slider(50, 150, 200, 0.1, 3.0, l[2], "Arm 3 Length")
slider4 = Slider(50, 200, 200, 0.1, 3.0, l[3], "Arm 4 Length")

#screen coordinate conversion functions
def to_screen(coord):
    return int(width / 2 + coord[0] * 100), int(height / 2 - coord[1] * 100)

def from_screen(coord):
    return np.array([(coord[0] - width / 2) / 100, -(coord[1] - height / 2) / 100])

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        #handle slider events
        slider1.handle_event(event)
        slider2.handle_event(event)
        slider3.handle_event(event)
        slider4.handle_event(event)

    #update segment lengths from sliders
    l[0] = slider1.value
    l[1] = slider2.value
    l[2] = slider3.value
    l[3] = slider4.value

    #get mouse position
    mouse_pos = pygame.mouse.get_pos()
    target = from_screen(mouse_pos)

    #initialize positions
    positions = [B.copy()]
    for i in range(1, len(l) + 1):
        # extend positions along x-axis
        positions.append(positions[i - 1] + np.array([l[i - 1], 0]))

    #make background white
    screen.fill((255, 255, 255))

    try:
        #apply FABRIK algorithm
        positions = fabrik(target, positions, l)

        #draw arm segments
        for i in range(len(positions) - 1):
            pygame.draw.line(screen, (0, 0, 0), to_screen(positions[i]), to_screen(positions[i + 1]), 3)

        #draw joints and target
        colors = [(0, 0, 255), (0, 255, 0), (255, 165, 0), (255, 0, 0), (128, 0, 128)]
        for i, pos in enumerate(positions):
            pygame.draw.circle(screen, colors[i % len(colors)], to_screen(pos), 5)
        pygame.draw.circle(screen, (128, 0, 128), mouse_pos, 5)

    except:
        pass  #handle any numerical errors easily

    # draw sliders
    slider1.draw(screen)
    slider2.draw(screen)
    slider3.draw(screen)
    slider4.draw(screen)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()