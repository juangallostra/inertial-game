import pygame, sys
from pygame.locals import *

from constants import *
from track import *
from entities import *


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
    track.generate_track(debug=debug, draw_checkpoints_in_track=draw_checkpoints_in_track) 

    # Player
    player = PlayerCar(RED, *track.get_track_start()[1])

    pygame.display.set_caption(TITLE)

    while True: # main loop
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                player.handle_keys(event)
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        player.render(screen)
        pygame.display.update()

if __name__ == '__main__':
    # rn.seed(rn.choice(COOL_TRACK_SEEDS))
    main(debug=False, draw_checkpoints_in_track=False)