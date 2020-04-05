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

    track = GameTrackGenerator()

    # generate the track
    points = track.random_points()
    hull = ConvexHull(points)
    track_points = track.shape_track(track.get_track_points(hull, points))
    corner_points = track.get_corners_with_kerb(track_points)
    f_points = track.smooth_track(track_points)
    # get complete corners from keypoints
    corners = track.get_full_corners(f_points, corner_points)
    # draw the actual track (road, kerbs, starting grid)
    track.draw_track(screen, GREY, f_points, corners)
    # draw checkpoints
    checkpoints = track.get_checkpoints(f_points)
    if draw_checkpoints_in_track or debug:
        for checkpoint in checkpoints:
            track.draw_checkpoint(screen, f_points, checkpoint, debug)
    if debug:
        # draw the different elements that end up
        # making the track
        track.draw_points(screen, WHITE, points)
        track.draw_convex_hull(hull, screen, points, RED)
        track.draw_points(screen, BLUE, track_points)
        track.draw_lines_from_points(screen, BLUE, track_points)    
        track.draw_points(screen, BLACK, f_points)

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