import random
import pygame
import sys
from collections import defaultdict
from math import sqrt
import numpy as np
import imageio


class Constants:
    SCREEN_WIDTH = 1600
    SCREEN_HEIGHT = 1024
    NUM_PARTICLES = 3000
    TARGET_FPS = 45
    BOUNDARY_MUL = 0.5
    BOUNDARY_MIN_X = 5
    BOUNDARY_MAX_X_DIFF_FROM_WIDTH = 5
    BOUNDARY_MIN_Y = 5
    BOUNDARY_MAX_Y_DIFF_FROM_HEIGHT = 5
    BALL_COLOR = (0, 102, 255)
    BALL_RADIUS = 2
    GRAV_X = 0
    GRAV_Y = 0.1
    INFLUENCE_RADIUS = 40
    TARGET_DENSITY = 4
    K_DENSITY_TO_PRESSURE = 0.5
    K_NEAR_DENSITY_TO_PRESSURE = 0.5


class Ball:
    def __init__(self, pos_x, pos_y, vel_x, vel_y):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.vel_x = vel_x
        self.vel_y = vel_y

    def dist(self, other_ball):
        return sqrt((other_ball.pos_x - self.pos_x)**2 + (other_ball.pos_y - self.pos_y)**2)


class Simulator:
    def __init__(self, width, height, univ_size):
        self.spatial_hash = None
        self.running = False
        self.width = width
        self.height = height
        self.univ_size = univ_size
        self.grav_x = Constants.GRAV_X
        self.grav_y = Constants.GRAV_Y
        self.universe = []
        self.create_universe(univ_size)

    def start(self):
        self.running = True

    def end(self):
        self.running = False

    def resize(self, new_width, new_height):
        self.width = new_width
        self.height = new_height

    def create_universe(self, univ_size):
        for i in range(univ_size):
            pos_x = random.random() * self.width
            pos_y = random.random() * self.height
            vel_x = random.random() * 2 - 1  # between -1 and 1
            vel_y = random.random() * 2 - 1
            self.universe.append(Ball(pos_x, pos_y, vel_x, vel_y))

    def draw(self, surface):
        for ball in self.universe:
            pygame.draw.circle(
                surface,
                Constants.BALL_COLOR,
                (int(ball.pos_x), int(ball.pos_y)),
                Constants.BALL_RADIUS
            )

    def get_spatial_hash(self):
        """
        creates a spatial hash and stores it in self.spatial_hash
        :return:
        """
        spatial_hash = defaultdict(list)
        for ball in self.universe:
            ball.cell_x = ball.pos_x // Constants.INFLUENCE_RADIUS
            ball.cell_y = ball.pos_y // Constants.INFLUENCE_RADIUS
            spatial_hash[(ball.cell_x, ball.cell_y)].append(ball)
        self.spatial_hash = spatial_hash

    def get_neighbours(self, ball):
        """
        gets the neighbours of each ball ignoring itself and stores it as a variable of the ball
        :param ball:
        :return:
        """
        neighbours = []
        for i in (-1,0,1):
            for j in (-1,0,1):
                if i == 0 and j == 0:
                    for other_ball in self.spatial_hash[(ball.cell_x + i, ball.cell_y + j)]:
                        if other_ball is ball:
                            continue
                        neighbours.append(other_ball)
                else:
                    neighbours.extend(self.spatial_hash[(ball.cell_x + i, ball.cell_y + j)])
        ball.neighbours = neighbours

    def update(self, dt=1):
        if not self.running:
            return

        for ball in self.universe:
            # apply gravity
            ball.vel_y += dt * self.grav_y
            ball.vel_x += dt * self.grav_x


        self.apply_viscosity(dt)


        for ball in self.universe:
            # saves previous position of each ball
            ball.pos_x_prev = ball.pos_x
            ball.pos_y_prev = ball.pos_y
            ball.pos_x += dt * ball.vel_x
            ball.pos_y += dt * ball.vel_y

        self.adjust_springs(dt)
        self.apply_spring_displacements(dt)
        self.double_density_relaxation(dt)
        self.resolve_collisions(dt)

        for ball in self.universe:
            # updates velocity with change in position
            ball.vel_x = (ball.pos_x - ball.pos_x_prev) / dt
            ball.vel_y = (ball.pos_y - ball.pos_y_prev) / dt


    def apply_viscosity(self, dt):
        pass

    def adjust_springs(self, dt):
        pass

    def apply_spring_displacements(self, dt):
        pass

    def double_density_relaxation(self, dt):
        self.get_spatial_hash()
        for ball in self.universe:
            ball.density = 0
        for ball in self.universe:
            ball.density_near = 0
            self.get_neighbours(ball)
            for other_ball in ball.neighbours:
                q = ball.dist(other_ball) / Constants.INFLUENCE_RADIUS
                if q < 1:
                    ball.density += (1-q)**2
                    ball.density_near += (1-q)**3
            ball.pressure = (ball.density - Constants.TARGET_DENSITY) * Constants.K_DENSITY_TO_PRESSURE
            ball.pressure_near = ball.density_near * Constants.K_NEAR_DENSITY_TO_PRESSURE
            dx = 0
            dy = 0
            for other_ball in ball.neighbours:
                q = ball.dist(other_ball) / Constants.INFLUENCE_RADIUS
                rij_x = other_ball.pos_x - ball.pos_x
                rij_y = other_ball.pos_y - ball.pos_y
                if q < 1:
                    D_x = dt**2 * (ball.pressure * (1-q) + ball.pressure_near * (1-q)**2) * rij_x / (ball.dist(other_ball) + 1e-9)
                    D_y = dt**2 * (ball.pressure * (1 - q) + ball.pressure_near * (1 - q)**2) * rij_y / (ball.dist(other_ball) + 1e-9)
                    other_ball.pos_x += D_x / 2
                    other_ball.pos_y += D_y / 2
                    dx -= D_x / 2
                    dy -= D_y / 2
            ball.pos_x += dx
            ball.pos_y += dy

    def resolve_collisions(self, dt):
        for ball in self.universe:
            if ball.pos_x < Constants.BOUNDARY_MIN_X:
                ball.pos_x += Constants.BOUNDARY_MUL * (Constants.BOUNDARY_MIN_X - ball.pos_x)
            elif ball.pos_x > self.width - Constants.BOUNDARY_MAX_X_DIFF_FROM_WIDTH:
                ball.pos_x += Constants.BOUNDARY_MUL * (
                            self.width - Constants.BOUNDARY_MAX_X_DIFF_FROM_WIDTH - ball.pos_x)

            if ball.pos_y < Constants.BOUNDARY_MIN_Y:
                ball.pos_y += Constants.BOUNDARY_MUL * (Constants.BOUNDARY_MIN_Y - ball.pos_y)
            elif ball.pos_y > self.height - Constants.BOUNDARY_MAX_Y_DIFF_FROM_HEIGHT:
                ball.pos_y += Constants.BOUNDARY_MUL * (
                            self.height - Constants.BOUNDARY_MAX_Y_DIFF_FROM_HEIGHT - ball.pos_y)


def main():

    pygame.init()

    WIDTH, HEIGHT = Constants.SCREEN_WIDTH, Constants.SCREEN_HEIGHT

    surface = pygame.Surface((WIDTH, HEIGHT))

    sim = Simulator(WIDTH, HEIGHT, univ_size=Constants.NUM_PARTICLES)
    sim.start()

    FPS = 60
    DURATION_SECONDS = 15
    TOTAL_FRAMES = FPS * DURATION_SECONDS
    dt = 1.0

    frames = []

    print("Simulating and capturing frames...")

    for frame in range(TOTAL_FRAMES):
        sim.update(dt)

        surface.fill((0, 0, 0))
        sim.draw(surface)

        # Convert surface → array (H, W, 3)
        frame_array = pygame.surfarray.array3d(surface)
        frame_array = np.transpose(frame_array, (1, 0, 2))  # correct orientation

        frames.append(frame_array)

        if frame % 30 == 0:
            print(f"Captured frame {frame}/{TOTAL_FRAMES}")

    print("Saving mp4...")

    imageio.mimsave(
        "sph_simulation.mp4",
        frames,
        fps=60,
        codec="libx264"
    )

    pygame.quit()
    print("Saved sph_simulation.mp4")



if __name__ == "__main__":
    main()