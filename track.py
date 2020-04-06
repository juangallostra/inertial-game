import pygame, sys
from pygame.locals import *
import math
import random as rn
import numpy as np
import scipy as sc
from scipy.spatial import ConvexHull
from scipy import interpolate

from constants import *

class GameTrackGenerator():
    def __init__(self, screen):
        self._track_points = []
        self._checkpoints = []
        self._screen = screen

    def generate_track(self, debug=False, draw_checkpoints_in_track=False):
        # generate the track
        points = self.random_points()
        hull = ConvexHull(points)
        track_points = self.shape_track(self.get_track_points(hull, points))
        corner_points = self.get_corners_with_kerb(track_points)
        f_points = self.smooth_track(track_points)
        # get complete corners from keypoints
        corners = self.get_full_corners(f_points, corner_points)
        # draw the actual track (road, kerbs, starting grid)
        self.draw_track(self._screen, GREY, f_points, corners)
        # draw checkpoints
        checkpoints = self.get_checkpoints(f_points)
        if draw_checkpoints_in_track or debug:
            for checkpoint in checkpoints:
                self.draw_checkpoint(self._screen, f_points, checkpoint, debug)
        if debug:
            # draw the different elements that end up
            # making the track
            self.draw_points(self._screen, WHITE, points)
            self.draw_convex_hull(hull, self._screen, points, RED)
            self.draw_points(self._screen, BLUE, track_points)
            self.draw_lines_from_points(self._screen, BLUE, track_points)    
            self.draw_points(self._screen, BLACK, f_points)

    def get_track_points(self):
        return self._track_points

    def get_checkpoints(self):
        return self._checkpoints

    # track generation methods
    def random_points(self, min=MIN_POINTS, max=MAX_POINTS, margin=MARGIN, min_distance=MIN_DISTANCE):
        point_count = rn.randrange(min, max+1, 1)
        points = []
        for i in range(point_count):
            x = rn.randrange(margin, WIDTH - margin + 1, 1)
            y = rn.randrange(margin, HEIGHT -margin + 1, 1)
            distances = list(
                filter(
                    lambda x: x < min_distance, [math.sqrt((p[0]-x)**2 + (p[1]-y)**2) for p in points]
                )
            )
            if len(distances) == 0:
                points.append((x, y))
        return np.array(points)

    def get_track_points(self, hull, points):
        # get the original points from the random 
        # set that will be used as the track starting shape
        return np.array([points[hull.vertices[i]] for i in range(len(hull.vertices))])

    def make_rand_vector(self, dims):
        vec = [rn.gauss(0, 1) for i in range(dims)]
        mag = sum(x**2 for x in vec) ** .5
        return [x/mag for x in vec]

    def shape_track(
        self,
        track_points,
        difficulty=DIFFICULTY,
        max_displacement=MAX_DISPLACEMENT,
        margin=MARGIN
    ):
        track_set = [[0,0] for i in range(len(track_points)*2)] 
        for i in range(len(track_points)):
            displacement = math.pow(rn.random(), difficulty) * max_displacement
            disp = [displacement * i for i in self.make_rand_vector(2)]
            track_set[i*2] = track_points[i]
            track_set[i*2 + 1][0] = int((track_points[i][0] + track_points[(i+1)%len(track_points)][0]) / 2 + disp[0])
            track_set[i*2 + 1][1] = int((track_points[i][1] + track_points[(i+1)%len(track_points)][1]) / 2 + disp[1])
        for i in range(3):
            track_set = self.fix_angles(track_set)
            track_set = self.push_points_apart(track_set)
        # push any point outside screen limits back again
        final_set = []
        for point in track_set:
            if point[0] < margin:
                point[0] = margin
            elif point[0] > (WIDTH - margin):
                point[0] = WIDTH - margin
            if point[1] < margin:
                point[1] = margin
            elif point[1] > HEIGHT - margin:
                point[1] = HEIGHT - margin
            final_set.append(point)
        return final_set

    def push_points_apart(self, points, distance=DISTANCE_BETWEEN_POINTS):
        # distance might need some tweaking
        distance2 = distance * distance 
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                p_distance =  math.sqrt((points[i][0]-points[j][0])**2 + (points[i][1]-points[j][1])**2)
                if p_distance < distance:
                    dx = points[j][0] - points[i][0];  
                    dy = points[j][1] - points[i][1];  
                    dl = math.sqrt(dx*dx + dy*dy);  
                    dx /= dl;  
                    dy /= dl;  
                    dif = distance - dl;  
                    dx *= dif;  
                    dy *= dif;  
                    points[j][0] = int(points[j][0] + dx);  
                    points[j][1] = int(points[j][1] + dy);  
                    points[i][0] = int(points[i][0] - dx);  
                    points[i][1] = int(points[i][1] - dy);  
        return points

    def fix_angles(self, points, max_angle=MAX_ANGLE):
        for i in range(len(points)):
            if i > 0:
                prev_point = i - 1
            else:
                prev_point = len(points)-1
            next_point = (i+1) % len(points)
            px = points[i][0] - points[prev_point][0]
            py = points[i][1] - points[prev_point][1]
            pl = math.sqrt(px*px + py*py)
            px /= pl
            py /= pl
            nx = -(points[i][0] - points[next_point][0])
            ny = -(points[i][1] - points[next_point][1])
            nl = math.sqrt(nx*nx + ny*ny)
            nx /= nl
            ny /= nl  
            a = math.atan2(px * ny - py * nx, px * nx + py * ny)
            if (abs(math.degrees(a)) <= max_angle):
                continue
            diff = math.radians(max_angle * math.copysign(1,a)) - a
            c = math.cos(diff)
            s = math.sin(diff)
            new_x = (nx * c - ny * s) * nl
            new_y = (nx * s + ny * c) * nl
            points[next_point][0] = int(points[i][0] + new_x)
            points[next_point][1] = int(points[i][1] + new_y)
        return points

    def get_corners_with_kerb(
        self,
        points,
        min_kerb_angle=MIN_KERB_ANGLE,
        max_kerb_angle=MAX_KERB_ANGLE
    ):
        require_kerb = []
        for i in range(len(points)):
            if i > 0:
                prev_point = i - 1
            else:
                prev_point = len(points)-1
            next_point = (i+1) % len(points)
            px = points[i][0] - points[prev_point][0]
            py = points[i][1] - points[prev_point][1]
            pl = math.sqrt(px*px + py*py)
            px /= pl
            py /= pl
            nx = -(points[i][0] - points[next_point][0])
            ny = -(points[i][1] - points[next_point][1])
            nl = math.sqrt(nx*nx + ny*ny)
            nx /= nl
            ny /= nl 
            # a = math.atan2(px * ny - py * nx, px * nx + py * ny)
            a = math.atan(px * ny - py * nx)
            if (min_kerb_angle <= abs(math.degrees(a)) <= max_kerb_angle):
                continue
            require_kerb.append(points[i])
        return require_kerb

    def smooth_track(self, track_points):
        x = np.array([p[0] for p in track_points])
        y = np.array([p[1] for p in track_points])

        # append the starting x,y coordinates
        x = np.r_[x, x[0]]
        y = np.r_[y, y[0]]

        # fit splines to x=f(u) and y=g(u), treating both as periodic. also note that s=0
        # is needed in order to force the spline fit to pass through all the input points.
        tck, u = interpolate.splprep([x, y], s=0, per=True)

        # evaluate the spline fits for # points evenly spaced distance values
        xi, yi = interpolate.splev(np.linspace(0, 1, SPLINE_POINTS), tck)
        return [(int(xi[i]), int(yi[i])) for i in range(len(xi))]

    def get_full_corners(self, track_points, corners):
        # get full range of points that conform the corner
        offset = FULL_CORNER_NUM_POINTS
        corners_in_track = self.get_corners_from_kp(track_points, corners)
        # for each corner keypoint in smoothed track, 
        # get the set of points that make the corner.
        # This are the offset previous and offset next points
        f_corners = []
        for corner in corners_in_track:
            # get kp index
            i = track_points.index(corner)
            # build temp list to get set of points
            tmp_track_points = track_points + track_points + track_points
            f_corner = tmp_track_points[i+len(track_points)-1-offset:i+len(track_points)-1+offset]
            f_corners.append(f_corner)
        return f_corners

    def get_corners_from_kp(self, complete_track, corner_kps):
        # for each detected corner find closest point in final track (smoothed track)
        return [self.find_closest_point(complete_track, corner) for corner in corner_kps]

    def find_closest_point(self, points, keypoint):
        min_dist = None
        closest_point = None
        for p in points:
            dist = math.hypot(p[0]-keypoint[0], p[1]-keypoint[1])
            if min_dist is None or dist < min_dist:
                min_dist = dist
                closest_point = p
        return closest_point

    def get_checkpoints(self, track_points, n_checkpoints=N_CHECKPOINTS):
        # get step between checkpoints
        checkpoint_step = len(track_points) // n_checkpoints
        # get checkpoint track points
        checkpoints = []
        for i in range(N_CHECKPOINTS):
            index = i * checkpoint_step
            checkpoints.append(track_points[index])
        return checkpoints

    ####
    ## drawing functions
    ####
    def draw_points(self, surface, color, points):
        for p in points:
            self.draw_single_point(surface, color, p)

    def draw_convex_hull(self, hull, surface, points, color):
        for i in range(len(hull.vertices)-1):
            self.draw_single_line(surface, color, points[hull.vertices[i]], points[hull.vertices[i+1]])
            # close the polygon
            if i == len(hull.vertices) - 2:
                self.draw_single_line(
                    surface,
                    color,
                    points[hull.vertices[0]],
                    points[hull.vertices[-1]]
                )

    def draw_lines_from_points(self, surface, color, points):
        for i in range(len(points)-1):
            self.draw_single_line(surface, color, points[i], points[i+1])
            # close the polygon
            if i == len(points) - 2:
                self.draw_single_line(
                    surface,
                    color,
                    points[0],
                    points[-1]
                )

    def draw_single_point(self, surface, color, pos, radius=2):
        pygame.draw.circle(surface, color, pos, radius)

    def draw_single_line(self, surface, color, init, end):
        pygame.draw.line(surface, color, init, end)

    def draw_track(self, surface, color, points, corners):
        radius = TRACK_WIDTH // 2
        # draw kerbs
        self.draw_corner_kerbs(surface, corners, radius)
        # draw track
        chunk_dimensions = (radius * 2, radius * 2)
        for point in points:
            blit_pos = (point[0] - radius, point[1] - radius)
            track_chunk = pygame.Surface(chunk_dimensions, pygame.SRCALPHA)
            pygame.draw.circle(track_chunk, color, (radius, radius), radius)
            surface.blit(track_chunk, blit_pos)
        starting_grid = self.draw_starting_grid(radius*2)
        # rotate and place starting grid
        offset = TRACK_POINT_ANGLE_OFFSET
        vec_p = [points[offset][1] - points[0][1], -(points[offset][0] - points[0][0])]
        n_vec_p = [
            vec_p[0] / math.hypot(vec_p[0], vec_p[1]),
            vec_p[1] / math.hypot(vec_p[0], vec_p[1])
        ]
        # compute angle
        angle = math.degrees(math.atan2(n_vec_p[1], n_vec_p[0]))
        rot_grid = pygame.transform.rotate(starting_grid, -angle)
        start_pos = (
            points[0][0] - math.copysign(1, n_vec_p[0])*n_vec_p[0] * radius,
            points[0][1] - math.copysign(1, n_vec_p[1])*n_vec_p[1] * radius
        )    
        surface.blit(rot_grid, start_pos)

    def draw_starting_grid(self, track_width):
        tile_height = START_TILE_HEIGHT # Move outside
        tile_width = START_TILE_WIDTH # Move outside
        grid_tile = pygame.image.load(STARTING_GRID_TILE)
        starting_grid = pygame.Surface((track_width, tile_height), pygame.SRCALPHA)
        for i in range(track_width // tile_height):
            position = (i*tile_width, 0)
            starting_grid.blit(grid_tile, position)
        return starting_grid

    def draw_checkpoint(self, track_surface, points, checkpoint, debug=False):
        # given the main point of a checkpoint, compute and draw the checkpoint box
        margin = CHECKPOINT_MARGIN
        radius = TRACK_WIDTH // 2 + margin
        offset = CHECKPOINT_POINT_ANGLE_OFFSET
        check_index = points.index(checkpoint)
        vec_p = [
            points[check_index + offset][1] - points[check_index][1],
            -(points[check_index+offset][0] - points[check_index][0])
        ]
        n_vec_p = [
            vec_p[0] / math.hypot(vec_p[0], vec_p[1]),
            vec_p[1] / math.hypot(vec_p[0], vec_p[1])
        ]
        # compute angle
        angle = math.degrees(math.atan2(n_vec_p[1], n_vec_p[0]))
        # draw checkpoint
        checkpoint = self.draw_rectangle((radius*2, 5), BLUE, line_thickness=1, fill=False)
        rot_checkpoint = pygame.transform.rotate(checkpoint, -angle)
        if debug:
            rot_checkpoint.fill(RED)
        check_pos = (
            points[check_index][0] - math.copysign(1, n_vec_p[0])*n_vec_p[0] * radius,
            points[check_index][1] - math.copysign(1, n_vec_p[1])*n_vec_p[1] * radius
        )    
        track_surface.blit(rot_checkpoint, check_pos)

    def draw_rectangle(self, dimensions, color, line_thickness=1, fill=False):
        filled = line_thickness
        if fill:
            filled = 0
        rect_surf = pygame.Surface(dimensions, pygame.SRCALPHA)
        pygame.draw.rect(rect_surf, color, (0, 0, dimensions[0], dimensions[1]), filled)
        return rect_surf

    def draw_corner_kerbs(self, track_surface, corners, track_width):
        # rotate and place kerbs
        step = STEP_TO_NEXT_KERB_POINT
        offset = KERB_POINT_ANGLE_OFFSET
        correction_x = KERB_PLACEMENT_X_CORRECTION
        correction_y = KERB_PLACEMENT_Y_CORRECTION
        for corner in corners:
            temp_corner = corner + corner
            last_kerb = None
            for i in range(0, len(corner), step):
                # parallel vector
                vec_p = [
                    temp_corner[i+offset][0] - temp_corner[i][0],
                    temp_corner[i+offset][1] - temp_corner[i][1]
                ]
                n_vec_p = [
                    vec_p[0] / math.hypot(vec_p[0], vec_p[1]),
                    vec_p[1] / math.hypot(vec_p[0], vec_p[1])
                ]
                # perpendicular vector
                vec_perp = [
                    temp_corner[i+offset][1] - temp_corner[i][1],
                    -(temp_corner[i+offset][0] - temp_corner[i][0])
                ]
                n_vec_perp = [
                    vec_perp[0] / math.hypot(vec_perp[0], vec_perp[1]),
                    vec_perp[1] / math.hypot(vec_perp[0], vec_perp[1])
                ]
                # compute angle
                angle = math.degrees(math.atan2(n_vec_p[1], n_vec_p[0]))
                kerb = self.draw_single_kerb()
                rot_kerb = pygame.transform.rotate(kerb, -angle)
                m_x = 1
                m_y = 1
                if angle > 180:
                    m_x = -1
                start_pos = (
                    corner[i][0] + m_x * n_vec_perp[0] * track_width - correction_x, 
                    corner[i][1] + m_y * n_vec_perp[1] * track_width - correction_y
                )
                if last_kerb is None:
                    last_kerb = start_pos
                else:
                    if math.hypot(
                        start_pos[0] - last_kerb[0],
                        start_pos[1] - last_kerb[1]
                    ) >= track_width:
                        continue
                last_kerb = start_pos
                track_surface.blit(rot_kerb, start_pos)

    def draw_single_kerb(self):
        tile_height = KERB_TILE_HEIGHT # Move outside
        tile_width = KERB_TILE_WIDTH # Move outside
        kerb_tile = pygame.image.load(KERB_TILE)
        kerb = pygame.Surface((tile_width, tile_height), pygame.SRCALPHA)
        kerb.blit(kerb_tile, (0, 0))
        return kerb
