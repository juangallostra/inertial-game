import pygame, sys
from pygame.locals import *

from constants import *
from track import *
from entities import *


####
## Main function
####
def main(debug=True, draw_checkpoints_in_track=True, draw_trajectory=True):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(TITLE)

    # The track already draws itself on the screen
    track = GameTrackGenerator(screen)
    track.generate_track(debug=debug, draw_checkpoints_in_track=draw_checkpoints_in_track) 

    # Player
    player_movement_keys = {
        LEFT:pygame.K_LEFT,
        RIGHT:pygame.K_RIGHT,
        UP:pygame.K_UP, 
        DOWN:pygame.K_DOWN
    }
    player = PlayerCar(player_movement_keys, RED, *track.get_track_start()[1])


    while True: # main loop
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                player.handle_keys(event)
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        if not draw_trajectory:
            track.draw_track()
        player.render(screen)
        pygame.display.update()

if __name__ == '__main__':
    # rn.seed(rn.choice(COOL_TRACK_SEEDS))
    main(debug=False, draw_checkpoints_in_track=False, draw_trajectory=True)