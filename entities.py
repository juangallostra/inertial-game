import pygame
from pygame.locals import *
from constants import *

class PlayerCar():
    def __init__(self, keys, color, x, y, inertia=[0,0]):
        # Keys should be a dict with keys left, right, up, down
        # set movement keys
        self._keys = keys
        self._inertia = inertia
        self._x = x
        self._y = y
        self._color = color
        self._size = (6, 6)

    def draw_single_line(self, surface, color, init, end):
        pygame.draw.line(surface, color, init, end)

    def render(self, surface):
        rect = pygame.Surface((5,5), pygame.SRCALPHA)
        pygame.draw.rect(rect, self._color, (0, 0, self._size[0], self._size[1]), 0)
        surface.blit(rect, (self._x, self._y))
        self.draw_single_line(
            surface,
            self._color,
            (
                self._x + int(self._size[0]/2),
                self._y
            ),
            (
                self._x + self._inertia[0] + int(self._size[0]/2),
                self._y + self._inertia[1]
            )
        )

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def move(self, dx, dy):
        self._x += dx
        self._y += dy

    def handle_keys(self, keypress):
        # update inertia and move
        # if keypress.key == pygame.K_LEFT:
        if keypress.key == self._keys[LEFT]:
            self._inertia[0] -= 1
            self.move(*self._inertia)
        # elif keypress.key == pygame.K_RIGHT:
        elif keypress.key == self._keys[RIGHT]:
            self._inertia[0] += 1
            self.move(*self._inertia)
        # elif keypress.key == pygame.K_UP:
        elif keypress.key == self._keys[UP]:
            self._inertia[1] -= 1
            self.move(*self._inertia)
        # elif keypress.key == pygame.K_DOWN:
        elif keypress.key == self._keys[DOWN]:
            self._inertia[1] += 1
            self.move(*self._inertia)