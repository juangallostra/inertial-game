import pygame, sys
from pygame.locals import *
import math
import random as rn
import numpy as np
import scipy as sc
from scipy.spatial import ConvexHull
from scipy import interpolate

from constants import *

from track import *

####
## Main function
####
def main(debug=True, draw_checkpoints_in_track=True):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    background_color = GRASS_GREEN
    screen.fill(background_color)

    # The track already draws itself on the screen
    track = GameTrackGenerator(screen)
    track.generate_track(debug=False, draw_checkpoints_in_track=False) 

    pygame.display.set_caption(TITLE)
    while True: # main loop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update()

if __name__ == '__main__':
    # rn.seed(rn.choice(COOL_TRACK_SEEDS))
    main(debug=False, draw_checkpoints_in_track=False)