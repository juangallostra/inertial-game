import pygame
from pygame.locals import *

class PlayerCar():
    def __init__(self, color, x, y, inertia=[0,0]):
        # set movement keys
        self._inertia = inertia
        self._x = x
        self._y = y
        self._color = color

    def render(self, surface):
        rect = pygame.Surface((5,5), pygame.SRCALPHA)
        pygame.draw.rect(rect, self._color, (0, 0, 5, 5), 0)
        surface.blit(rect, (self._x, self._y))

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def move(self, dx, dy):
        self._x += dx
        self._y += dy

    def handle_keys(self, keypress):
        # update inertia and move
        if keypress.key == pygame.K_LEFT:
            self._inertia[0] -= 1
            self.move(*self._inertia)
        elif keypress.key == pygame.K_RIGHT:
            self._inertia[0] += 1
            self.move(*self._inertia)
        elif keypress.key == pygame.K_UP:
            self._inertia[1] -= 1
            self.move(*self._inertia)
        elif keypress.key == pygame.K_DOWN:
            self._inertia[1] += 1
            self.move(*self._inertia)