#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import cv2
import re
import glob
import os
import sys
import time
from random import randint
import pygame
import numpy as np
import hashlib
import uuid
import json
import math
from progress.bar import Bar

HOME = os.path.join("C:\\Users\\Caco\\Desktop\\UNIVERSIDAD\\CUARTO\\TFG\\Parser_Audios\\Parser_Audio","dataset","parse")

ID = str(uuid.uuid4())

if not os.path.exists("dataset"):
    os.makedirs("dataset")

if not os.path.exists(HOME):
    os.makedirs(HOME)

videos = {}
    
FILE_NAME_COUNTER = 0


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc
import argparse

pygame.init()

counter = 0

PIXELS_PER_METER = 12

COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_BUTTER_1 = pygame.Color(237, 212, 0)
COLOR_BUTTER_2 = pygame.Color(196, 160, 0)

COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_ORANGE_1 = pygame.Color(245, 121, 0)
COLOR_ORANGE_2 = pygame.Color(209, 92, 0)

COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110)
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17)
COLOR_CHOCOLATE_2 = pygame.Color(143, 89, 2)

COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_CHAMELEON_1 = pygame.Color(115, 210, 22)
COLOR_CHAMELEON_2 = pygame.Color(78, 154, 6)

COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207)
COLOR_SKY_BLUE_1 = pygame.Color(52, 101, 164)
COLOR_SKY_BLUE_2 = pygame.Color(32, 74, 135)

COLOR_PLUM_0 = pygame.Color(173, 127, 168)
COLOR_PLUM_1 = pygame.Color(117, 80, 123)
COLOR_PLUM_2 = pygame.Color(92, 53, 102)

COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0)
COLOR_SCARLET_RED_2 = pygame.Color(164, 0, 0)

COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)


COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)

class MapImage(object):
    """Class encharged of rendering a 2D image from top view of a carla world. Please note that a cache system is used, so if the OpenDrive content
    of a Carla town has not changed, it will read and use the stored image if it was rendered in a previous execution"""

    def __init__(self, args, client, carla_world, carla_map, pixels_per_meter, tickDuration, show_triggers=False, show_connections=False, show_spawn_points=False):
        global counter
        """ Renders the map image generated based on the world, its map and additional flags that provide extra information about the road network"""
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0
        self.show_triggers = show_triggers
        self.show_connections = show_connections
        self.show_spawn_points = show_spawn_points

        waypoints = carla_map.generate_waypoints(2)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        # Maximum size of a Pygame surface
        width_in_pixels = (1 << 14) - 1

        # Adapt Pixels per meter to make world fit in surface
        surface_pixel_per_meter = int(width_in_pixels / self.width)
        if surface_pixel_per_meter > PIXELS_PER_METER:
            surface_pixel_per_meter = PIXELS_PER_METER

        self._pixels_per_meter = surface_pixel_per_meter
        width_in_pixels = int(self._pixels_per_meter * self.width)

        self.big_map_surface = pygame.Surface((width_in_pixels, width_in_pixels))

        # Load OpenDrive content
        opendrive_content = carla_map.to_opendrive()

        # Get hash based on content
        hash_func = hashlib.sha1()
        hash_func.update(opendrive_content.encode("UTF-8"))
        opendrive_hash = str(hash_func.hexdigest())

        # Build path for saving or loading the cached rendered map
        filename = carla_map.name.split('/')[-1] + "_FULL_RECORRIDO_" + ".tga"
        dirname = os.path.join(HOME, ID)
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        self.full_path = str(os.path.join(dirname, filename))


        if os.path.isfile(self.full_path):
            # Load Image
            self.big_map_surface = pygame.image.load(self.full_path)
        else:
            # Render map
            self.draw_road_map(
                self.big_map_surface,
                carla_world,
                carla_map,
                self.world_to_pixel,
                self.world_to_pixel_width)
            player = None
            for actor in carla_world.get_actors().filter('vehicle.tesla.model3'):
                if actor.attributes.get('role_name') == 'hero':
                    player = actor
            timeout = time.time()
            locations = []
            tickCounter = 0
            bar = Bar('Creando mapa y recorrido', max=tickDuration)
            while (tickCounter<tickDuration):
                locations.append(self.world_to_pixel(player.get_location()))
                try:
                    carla_world.tick()
                except Exception as e:
                    break
                bar.next()
                #time.sleep(0.1)
                tickCounter+=1
            
            print(len(locations))
            self.draw_solid_line(self.big_map_surface, COLOR_CHAMELEON_0, False, locations, 20)
            bar.finish()
            # Save rendered map for next executions of same map
            pygame.image.save(self.big_map_surface, self.full_path)
            pygame.image.save_extended(self.big_map_surface, self.full_path.replace(".tga", ".png"))


        self.surface = self.big_map_surface

    def draw_solid_line(self, surface, color, closed, points, width):
        """Draws solid lines in a surface given a set of points, width and color"""
        if len(points) >= 2:
            pygame.draw.lines(surface, color, closed, points, width)

    def draw_road_map(self, map_surface, carla_world, carla_map, world_to_pixel, world_to_pixel_width):
        """Draws all the roads, including lane markings, arrows and traffic signs"""
        map_surface.fill(COLOR_ALUMINIUM_4)
        precision = 0.05

        def draw_solid_line(surface, color, closed, points, width):
            """Draws solid lines in a surface given a set of points, width and color"""
            if len(points) >= 2:
                pygame.draw.lines(surface, color, closed, points, width)

        def lane_marking_color_to_tango(lane_marking_color):
            """Maps the lane marking color enum specified in PythonAPI to a Tango Color"""
            tango_color = COLOR_BLACK

            if lane_marking_color == carla.LaneMarkingColor.White:
                tango_color = COLOR_ALUMINIUM_2

            elif lane_marking_color == carla.LaneMarkingColor.Blue:
                tango_color = COLOR_SKY_BLUE_0

            elif lane_marking_color == carla.LaneMarkingColor.Green:
                tango_color = COLOR_CHAMELEON_0

            elif lane_marking_color == carla.LaneMarkingColor.Red:
                tango_color = COLOR_SCARLET_RED_0

            elif lane_marking_color == carla.LaneMarkingColor.Yellow:
                tango_color = COLOR_ORANGE_0

            return tango_color



        def draw_broken_line(surface, color, closed, points, width):
            """Draws broken lines in a surface given a set of points, width and color"""
            # Select which lines are going to be rendered from the set of lines
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 20)) if n % 3 == 0]

            # Draw selected lines
            for line in broken_lines:
                pygame.draw.lines(surface, color, closed, line, width)

        def get_lane_markings(lane_marking_type, lane_marking_color, waypoints, sign):
            """For multiple lane marking types (SolidSolid, BrokenSolid, SolidBroken and BrokenBroken), it converts them
             as a combination of Broken and Solid lines"""
            margin = 0.25
            marking_1 = [world_to_pixel(lateral_shift(w.transform, sign * w.lane_width * 0.5)) for w in waypoints]
            if lane_marking_type == carla.LaneMarkingType.Broken or (lane_marking_type == carla.LaneMarkingType.Solid):
                return [(lane_marking_type, lane_marking_color, marking_1)]
            else:
                marking_2 = [world_to_pixel(lateral_shift(w.transform,
                                                          sign * (w.lane_width * 0.5 + margin * 2))) for w in waypoints]
                if lane_marking_type == carla.LaneMarkingType.SolidBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.BrokenBroken:
                    return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
                elif lane_marking_type == carla.LaneMarkingType.SolidSolid:
                    return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                            (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]

            return [(carla.LaneMarkingType.NONE, carla.LaneMarkingColor.Other, [])]

        def draw_lane(surface, lane, color):
            """Renders a single lane in a surface and with a specified color"""
            for side in lane:
                lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
                lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]

                polygon = lane_left_side + [x for x in reversed(lane_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]

                if len(polygon) > 2:
                    pygame.draw.polygon(surface, color, polygon, 5)
                    pygame.draw.polygon(surface, color, polygon)

        def draw_lane_marking(surface, waypoints):
            """Draws the left and right side of lane markings"""
            # Left Side
            draw_lane_marking_single_side(surface, waypoints[0], -1)

            # Right Side
            draw_lane_marking_single_side(surface, waypoints[1], 1)

        def draw_lane_marking_single_side(surface, waypoints, sign):
            """Draws the lane marking given a set of waypoints and decides whether drawing the right or left side of
            the waypoint based on the sign parameter"""
            lane_marking = None

            marking_type = carla.LaneMarkingType.NONE
            previous_marking_type = carla.LaneMarkingType.NONE

            marking_color = carla.LaneMarkingColor.Other
            previous_marking_color = carla.LaneMarkingColor.Other

            markings_list = []
            temp_waypoints = []
            current_lane_marking = carla.LaneMarkingType.NONE
            for sample in waypoints:
                lane_marking = sample.left_lane_marking if sign < 0 else sample.right_lane_marking

                if lane_marking is None:
                    continue

                marking_type = lane_marking.type
                marking_color = lane_marking.color

                if current_lane_marking != marking_type:
                    # Get the list of lane markings to draw
                    markings = get_lane_markings(
                        previous_marking_type,
                        lane_marking_color_to_tango(previous_marking_color),
                        temp_waypoints,
                        sign)
                    current_lane_marking = marking_type

                    # Append each lane marking in the list
                    for marking in markings:
                        markings_list.append(marking)

                    temp_waypoints = temp_waypoints[-1:]

                else:
                    temp_waypoints.append((sample))
                    previous_marking_type = marking_type
                    previous_marking_color = marking_color

            # Add last marking
            last_markings = get_lane_markings(
                previous_marking_type,
                lane_marking_color_to_tango(previous_marking_color),
                temp_waypoints,
                sign)
            for marking in last_markings:
                markings_list.append(marking)

            # Once the lane markings have been simplified to Solid or Broken lines, we draw them
            for markings in markings_list:
                if markings[0] == carla.LaneMarkingType.Solid:
                    draw_solid_line(surface, markings[1], False, markings[2], 2)
                elif markings[0] == carla.LaneMarkingType.Broken:
                    draw_broken_line(surface, markings[1], False, markings[2], 2)

        def draw_arrow(surface, transform, color=COLOR_ALUMINIUM_2):
            """ Draws an arrow with a specified color given a transform"""
            transform.rotation.yaw += 180
            forward = transform.get_forward_vector()
            transform.rotation.yaw += 90
            right_dir = transform.get_forward_vector()
            end = transform.location
            start = end - 2.0 * forward
            right = start + 0.8 * forward + 0.4 * right_dir
            left = start + 0.8 * forward - 0.4 * right_dir

            # Draw lines
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [start, end]], 4)
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [left, start, right]], 4)

        def draw_traffic_signs(surface, font_surface, actor, color=COLOR_ALUMINIUM_2, trigger_color=COLOR_PLUM_0):
            """Draw stop traffic signs and its bounding box if enabled"""
            transform = actor.get_transform()
            waypoint = carla_map.get_waypoint(transform.location)

            angle = -waypoint.transform.rotation.yaw - 90.0
            font_surface = pygame.transform.rotate(font_surface, angle)
            pixel_pos = world_to_pixel(waypoint.transform.location)
            offset = font_surface.get_rect(center=(pixel_pos[0], pixel_pos[1]))
            surface.blit(font_surface, offset)

            # Draw line in front of stop
            forward_vector = carla.Location(waypoint.transform.get_forward_vector())
            left_vector = carla.Location(-forward_vector.y, forward_vector.x,
                                         forward_vector.z) * waypoint.lane_width / 2 * 0.7

            line = [(waypoint.transform.location + (forward_vector * 1.5) + (left_vector)),
                    (waypoint.transform.location + (forward_vector * 1.5) - (left_vector))]

            line_pixel = [world_to_pixel(p) for p in line]
            pygame.draw.lines(surface, color, True, line_pixel, 2)

            # Draw bounding box of the stop trigger
            if self.show_triggers:
                corners = Util.get_bounding_box(actor)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, trigger_color, True, corners, 2)

        # def draw_crosswalk(surface, transform=None, color=COLOR_ALUMINIUM_2):
        #     """Given two points A and B, draw white parallel lines from A to B"""
        #     a = carla.Location(0.0, 0.0, 0.0)
        #     b = carla.Location(10.0, 10.0, 0.0)

        #     ab = b - a
        #     length_ab = math.sqrt(ab.x**2 + ab.y**2)
        #     unit_ab = ab / length_ab
        #     unit_perp_ab = carla.Location(-unit_ab.y, unit_ab.x, 0.0)

        #     # Crosswalk lines params
        #     space_between_lines = 0.5
        #     line_width = 0.7
        #     line_height = 2

        #     current_length = 0
        #     while current_length < length_ab:

        #         center = a + unit_ab * current_length

        #         width_offset = unit_ab * line_width
        #         height_offset = unit_perp_ab * line_height
        #         list_point = [center - width_offset - height_offset,
        #                       center + width_offset - height_offset,
        #                       center + width_offset + height_offset,
        #                       center - width_offset + height_offset]

        #         list_point = [world_to_pixel(p) for p in list_point]
        #         pygame.draw.polygon(surface, color, list_point)
        #         current_length += (line_width + space_between_lines) * 2

        def lateral_shift(transform, shift):
            """Makes a lateral shift of the forward vector of a transform"""
            transform.rotation.yaw += 90
            return transform.location + shift * transform.get_forward_vector()

        def draw_topology(carla_topology, index):
            """ Draws traffic signs and the roads network with sidewalks, parking and shoulders by generating waypoints"""
            topology = [x[index] for x in carla_topology]
            topology = sorted(topology, key=lambda w: w.transform.location.z)
            set_waypoints = []
            for waypoint in topology:
                waypoints = [waypoint]

                # Generate waypoints of a road id. Stop when road id differs
                nxt = waypoint.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                    while nxt.road_id == waypoint.road_id:
                        waypoints.append(nxt)
                        nxt = nxt.next(precision)
                        if len(nxt) > 0:
                            nxt = nxt[0]
                        else:
                            break
                set_waypoints.append(waypoints)

                # Draw Shoulders, Parkings and Sidewalks
                PARKING_COLOR = COLOR_ALUMINIUM_4_5
                SHOULDER_COLOR = COLOR_ALUMINIUM_5
                SIDEWALK_COLOR = COLOR_ALUMINIUM_3

                shoulder = [[], []]
                parking = [[], []]
                sidewalk = [[], []]

                for w in waypoints:
                    # Classify lane types until there are no waypoints by going left
                    l = w.get_left_lane()
                    while l and l.lane_type != carla.LaneType.Driving:

                        if l.lane_type == carla.LaneType.Shoulder:
                            shoulder[0].append(l)

                        if l.lane_type == carla.LaneType.Parking:
                            parking[0].append(l)

                        if l.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[0].append(l)

                        l = l.get_left_lane()

                    # Classify lane types until there are no waypoints by going right
                    r = w.get_right_lane()
                    while r and r.lane_type != carla.LaneType.Driving:

                        if r.lane_type == carla.LaneType.Shoulder:
                            shoulder[1].append(r)

                        if r.lane_type == carla.LaneType.Parking:
                            parking[1].append(r)

                        if r.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[1].append(r)

                        r = r.get_right_lane()

                # Draw classified lane types
                draw_lane(map_surface, shoulder, SHOULDER_COLOR)
                draw_lane(map_surface, parking, PARKING_COLOR)
                draw_lane(map_surface, sidewalk, SIDEWALK_COLOR)

            # Draw Roads
            for waypoints in set_waypoints:
                waypoint = waypoints[0]
                road_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
                road_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

                polygon = road_left_side + [x for x in reversed(road_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]

                if len(polygon) > 2:
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon, 5)
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon)

                # Draw Lane Markings and Arrows
                if not waypoint.is_junction:
                    draw_lane_marking(map_surface, [waypoints, waypoints])
                    for n, wp in enumerate(waypoints):
                        if ((n + 1) % 400) == 0:
                            draw_arrow(map_surface, wp.transform)

        topology = carla_map.get_topology()
        draw_topology(topology, 0)

        if self.show_spawn_points:
            for sp in carla_map.get_spawn_points():
                draw_arrow(map_surface, sp, color=COLOR_CHOCOLATE_0)

        if self.show_connections:
            dist = 1.5

            def to_pixel(wp): return world_to_pixel(wp.transform.location)
            for wp in carla_map.generate_waypoints(dist):
                col = (0, 255, 255) if wp.is_junction else (0, 255, 0)
                for nxt in wp.next(dist):
                    pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(nxt), 2)
                if wp.lane_change & carla.LaneChange.Right:
                    r = wp.get_right_lane()
                    if r and r.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(r), 2)
                if wp.lane_change & carla.LaneChange.Left:
                    l = wp.get_left_lane()
                    if l and l.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(l), 2)

        actors = carla_world.get_actors()

        # Find and Draw Traffic Signs: Stops and Yields
        font_size = world_to_pixel_width(1)
        font = pygame.font.SysFont('Arial', font_size, True)

        stops = [actor for actor in actors if 'stop' in actor.type_id]
        yields = [actor for actor in actors if 'yield' in actor.type_id]

        stop_font_surface = font.render("STOP", False, COLOR_ALUMINIUM_2)
        stop_font_surface = pygame.transform.scale(
            stop_font_surface, (stop_font_surface.get_width(), stop_font_surface.get_height() * 2))

        yield_font_surface = font.render("YIELD", False, COLOR_ALUMINIUM_2)
        yield_font_surface = pygame.transform.scale(
            yield_font_surface, (yield_font_surface.get_width(), yield_font_surface.get_height() * 2))

        for ts_stop in stops:
            draw_traffic_signs(map_surface, stop_font_surface, ts_stop, trigger_color=COLOR_SCARLET_RED_1)

        for ts_yield in yields:
            draw_traffic_signs(map_surface, yield_font_surface, ts_yield, trigger_color=COLOR_ORANGE_1)

    def world_to_pixel(self, location, offset=(0, 0)):
        """Converts the world coordinates to pixel coordinates"""
        x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
        return [int(x - offset[0]), int(y - offset[1])]

    def world_to_pixel_width(self, width):
        """Converts the world units to pixel units"""
        return int(self.scale * self._pixels_per_meter * width)

    def scale_map(self, scale):
        """Scales the map surface"""
        if scale != self.scale:
            self.scale = scale
            width = int(self.big_map_surface.get_width() * self.scale)
            self.surface = pygame.transform.smoothscale(self.big_map_surface, (width, width))



def main():
    global counter, FILE_NAME_COUNTER, ID
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--start',
        metavar='S',
        default=0.0,
        type=float,
        help='starting time (default: 0.0)')
    argparser.add_argument(
        '-d', '--duration',
        metavar='D',
        default=0.0,
        type=float,
        help='duration (default: 0.0)')
    argparser.add_argument(
        '-f', '--recorder-filename',
        metavar='F',
        default=os.path.normpath(os.path.join(os.getcwd(),"dataset","records","autopilot","recording_10_27_2021T18_22_04.rec")),
        help='recorder filename (test1.log)')
    argparser.add_argument(
        '-c', '--camera',
        metavar='C',
        default=0,
        type=int,
        help='camera follows an actor (ex: 82)')
    argparser.add_argument(
        '-x', '--time-factor',
        metavar='X',
        default=1.0,
        type=float,
        help='time factor (default 1.0)')
    argparser.add_argument(
        '-i', '--ignore-hero',
        action='store_true',
        help='ignore hero vehicles')
    argparser.add_argument(
        '--spawn-sensors',
        action='store_true',
        help='spawn sensors in the replayed world')
    args = argparser.parse_args()

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        client.reload_world(reset_settings=True)
        ID = args.recorder_filename[args.recorder_filename.rindex(os.sep, args.recorder_filename.rindex(os.sep)-1)+1:args.recorder_filename.rindex(os.sep)] + args.recorder_filename[args.recorder_filename.rindex(os.sep)+1:args.recorder_filename.rindex(".")]
        # set the time factor for the replayer
        client.set_replayer_time_factor(args.time_factor)

        # set to ignore the hero vehicles or not
        client.set_replayer_ignore_hero(args.ignore_hero)
        sim_world = client.get_world()
        for actor in sim_world.get_actors():
            actor.destroy()

        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.03
        sim_world.apply_settings(settings)

        response = client.replay_file(args.recorder_filename, args.start, args.duration, args.camera, args.spawn_sensors)
        print(response)
        print(client.show_recorder_file_info(args.recorder_filename, False))
        ticks = (int(re.search("Frames\:.*", client.show_recorder_file_info(args.recorder_filename, False)).group(0).split(":")[1])) - 1
        print("Total_ticks: {}".format(ticks))
        durationSeconds = (float(re.search("Duration\:.*", client.show_recorder_file_info(args.recorder_filename, False)).group(0).split(": ")[1].replace(" seconds", "")))
        print("Duración total: {} s".format(durationSeconds))
        tickDuration = durationSeconds/settings.fixed_delta_seconds
        tickDuration-=1
        sim_world.tick()

        map_image = MapImage(args,
            client,
            carla_world=sim_world,
            carla_map=sim_world.get_map(),
            pixels_per_meter=PIXELS_PER_METER,
            tickDuration=tickDuration)


        client.stop_replayer(False)

        for actor in sim_world.get_actors():
            actor.destroy()
        
        # replay the session
        response = client.replay_file(args.recorder_filename, args.start, args.duration, args.camera, args.spawn_sensors)
        ticks = (int(re.search("Frames\:.*", client.show_recorder_file_info(args.recorder_filename, False)).group(0).split(":")[1])) - 1
        sim_world.tick()

        player = None
        for actor in sim_world.get_actors().filter('vehicle.tesla.model3'):
            print("hi")
            if actor.attributes.get('role_name') == 'hero':
                player = actor



        bound_x = 0.5 + player.bounding_box.extent.x
        bound_y = 0.5 + player.bounding_box.extent.y
        bound_z = 0.5 + player.bounding_box.extent.z

        _sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
        ]


        '''def showImage(frame, image):
            global counter, videos
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            #videos[frame].write(array)
            cv2.imwrite(os.path.join(HOME, ID, str(FILE_NAME_COUNTER)+"_"+frame+".png"), array)
            counter+=1

        _camera_transforms = [
                (carla.Transform(carla.Location(x=1, z=1.1), carla.Rotation()), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=+0.6, y=-bound_y*0.59, z=1.03), carla.Rotation(yaw=180+15)), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=+0.6, y=bound_y*0.59, z=1.03), carla.Rotation(yaw=180-15)), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=-bound_x, z=1.1), carla.Rotation(yaw=180)), carla.AttachmentType.Rigid)]


        bp_library = sim_world.get_blueprint_library()
        for item in _sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(1920))
                bp.set_attribute('image_size_y', str(1080))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(2.2))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
                             
        sensor_driver = player.get_world().spawn_actor(
                _sensors[0][-1],
                _camera_transforms[0][0],
                attach_to=player,
                attachment_type=_camera_transforms[0][1])

        sensor_driver.listen(lambda image: showImage("driver",image))

        sensor_back = player.get_world().spawn_actor(
                _sensors[0][-1],
                _camera_transforms[3][0],
                attach_to=player,
                attachment_type=_camera_transforms[0][1])

        sensor_back.listen(lambda image: showImage("back",image))

        sensor_left = player.get_world().spawn_actor(
                _sensors[0][-1],
                _camera_transforms[1][0],
                attach_to=player,
                attachment_type=_camera_transforms[0][1])

        sensor_left.listen(lambda image: showImage("left",image))

        sensor_right = player.get_world().spawn_actor(
                _sensors[0][-1],
                _camera_transforms[2][0],
                attach_to=player,
                attachment_type=_camera_transforms[0][1])

        sensor_right.listen(lambda image: showImage("right",image))
'''
        img = cv2.imread(map_image.full_path.replace(".tga", ".png"))

        carla_map = player.get_world().get_map()
        player.set_autopilot(True)


        def rotate_image(image, angle, x, y):
          rot_mat = cv2.getRotationMatrix2D((x,y), angle, 1.0)
          result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
          return result

        lastPosition=None
        crop_img=None
        tickCounter=0
        b_x = 0
        b_y = 0
        b_z = 0
        try:
            bar = Bar('Creando imagenes y datos', max=tickDuration)
            tickCounter = 0
            data_complete = []
            while (tickCounter<tickDuration):
                
                counter=0
                sim_world.tick()
                now = player.get_location()
                '''if(now!=lastPosition):
                    h, w = map_image._pixels_per_meter*15, map_image._pixels_per_meter*8
                    x, y = map_image.world_to_pixel(now)

                    crop_img = rotate_image(img, player.get_transform().rotation.yaw + 90, x, y)[int(y-math.floor(h/2)):int(y+math.ceil(h/2)), int(x-math.floor(w/2)):int(x+math.ceil(w/2))]                
                '''
                lastPosition = now
                control = player.get_control()
                acceleration = player.get_acceleration()
                location = player.get_location()
                geo_location = carla_map.transform_to_geolocation(location)
                rotation = player.get_transform().rotation
                #velocity = player.get_velocity()
                if tickCounter == 0:
                    b_x = location.x
                    b_y = location.y
                    b_z = location.z
                e_x = location.x - b_x
                e_y = location.y - b_y
                e_z = location.z - b_z
                real_speed = 3.6 * (math.sqrt(e_x**2 + e_y**2 + e_z**2) / 0.03)
                #real_speed_str = "{:.0f} km/h".format(real_speed)
                angularVelocity = player.get_angular_velocity()
                lightState = player.get_light_state()
                #print("\nLIGHT STATE = {}\n".format(lightState))

                # Para calcular el valor multiplicado por tickCounter hay que hacer lo siguiente: segundos_totales_video / frames_totales_replay
                data = {
                    "timestamp": round(tickCounter * 0.05421686746988, 2),
                    "control" : {
                        "throttle" : control.throttle,
                        "steer" : control.steer,
                        "brake" : control.brake,
                        "light_state" : lightState,
                        "hand_brake" : control.hand_brake,
                        "reverse" : control.reverse,
                        "manual_gear_shift" : control.manual_gear_shift,
                        "gear" : control.gear
                        },
                    "speed (km/h)" : round(real_speed, 0),
                    "speedLimit" : player.get_speed_limit(),
                    "trafficLightState" : str(player.get_traffic_light_state()),
                    "acceleration" : {
                        "x" : acceleration.x,
                        "y" : acceleration.y,
                        "z" : acceleration.z,
                        },
                    "location" : {
                        "x" : location.x,
                        "y" : location.y,
                        "z" : location.z,
                        },
                    "geoLocation" : {
                        "latitude" : geo_location.latitude,
                        "longitude" : geo_location.longitude,
                        "altitude" : geo_location.altitude,
                        },
                    "angularVelocity" : {
                        "x" : angularVelocity.x,
                        "y" : angularVelocity.y,
                        "z" : angularVelocity.z,
                        },
                    "rotation" : {
                        "x" : rotation.roll,
                        "y" : rotation.pitch,
                        "z" : rotation.yaw,
                        }
                }
                b_x = location.x
                b_y = location.y
                b_z = location.z
                data_complete.append(data)
                '''with open(os.path.join(HOME, ID, str(FILE_NAME_COUNTER)+"_DATA"+".json"), 'w') as outfile:
                    json.dump(data_complete, outfile, indent=4)
                '''

                #videos["MAP"].write(crop_img)
                #cv2.imwrite(os.path.join(HOME, ID, str(FILE_NAME_COUNTER)+"_MAP"+".png"), crop_img)
                '''while(counter<len(_camera_transforms)):
                    pass'''
                FILE_NAME_COUNTER+=1
                bar.next()
                tickCounter+=1

        finally:
            bar.finish()
            with open(os.path.join(HOME, ID, "SIM_DATA"+".json"), 'w') as outfile:
                json.dump(data_complete, outfile, indent=4)
            try:
                client.stop_replayer(False)
            except Exception as e:
                pass

            try:
                client.reload_world(reset_settings=True)
            except Exception as e:
                pass

                
    finally:
        pass


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
