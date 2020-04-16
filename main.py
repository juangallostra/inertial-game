import pygame, sys
from pygame.locals import *

from constants import *
from track import *
from entities import *

P1 = "p1"
P2 = "p2"

####
## Main function
####
def main(debug=True, draw_checkpoints_in_track=True, draw_trajectory=True, multiplayer=False):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(TITLE)

    # The track already draws itself on the screen
    track = GameTrackGenerator(screen)
    track.generate_track(debug=debug, draw_checkpoints_in_track=draw_checkpoints_in_track) 

    # Players
    player_movement_keys = {
        LEFT:pygame.K_LEFT,
        RIGHT:pygame.K_RIGHT,
        UP:pygame.K_UP, 
        DOWN:pygame.K_DOWN
    }
    player_2_movement_keys = {
        LEFT:pygame.K_a,
        RIGHT:pygame.K_d,
        UP:pygame.K_w,
        DOWN:pygame.K_s,
    }

    player = PlayerCar(player_movement_keys, RED, *track.get_track_start()[1])
    players = [player, player]
    turn_switch = {P1:P1, P1:P1}
    if multiplayer:
        player2 = PlayerCar(player_2_movement_keys, BLUE, *track.get_track_start()[1])
        players = [player, player2]
        turn_switch = {P1:P2, P2:P1}

    turn = P1
    
    def handle_turn(event, turn, players):
        if turn == P1:
            moved = players[0].handle_keys(event)
            if moved:
                return turn_switch[turn]
        elif turn==P2:
            moved = players[1].handle_keys(event)
            if moved:
                return turn_switch[turn]
        return turn
        
    while True: # main loop
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                turn = handle_turn(event, turn, players)
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        if not draw_trajectory:
            track.draw_track(debug=debug, draw_checkpoints_in_track=draw_checkpoints_in_track)
        for g_player in players:
            g_player.render(screen)
        pygame.display.update()

if __name__ == '__main__':
    # rn.seed(rn.choice(COOL_TRACK_SEEDS))
    main(debug=False, draw_checkpoints_in_track=False, draw_trajectory=True)
