class BaseObject():
    def __init__(self, x, y, angle):
        self._x = x
        self._y = y
        self._angle = angle

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def move(self, dx, dy):
        self._x += dx
        self._y += dy
