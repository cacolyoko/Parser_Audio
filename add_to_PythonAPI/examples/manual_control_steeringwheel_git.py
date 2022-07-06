#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function

from screeninfo import get_monitors
monitor = get_monitors()[0]

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import cv2
import glob
import os
import sys
import easygui as eg
import sounddevice as sd

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref



if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import *
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

#HOME = os.path.join(os.getcwd(),"dataset","records")
HOME = os.path.join("C:\\Users\\Caco\\Desktop\\UNIVERSIDAD\\CUARTO\\TFG\\Parser_Audios\\Parser_Audio","dataset","records")

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class joystick_handler(object):
    def __init__(self, id):
        self.id = id
        self.joy = pygame.joystick.Joystick(id)
        self.name = self.joy.get_name()
        self.joy.init()
        self.numaxes    = self.joy.get_numaxes()
        self.numballs   = self.joy.get_numballs()
        self.numbuttons = self.joy.get_numbuttons()
        self.numhats    = self.joy.get_numhats()

        self.axis = []
        for i in range(self.numaxes):
            self.axis.append(self.joy.get_axis(i))

        self.ball = []
        for i in range(self.numballs):
            self.ball.append(self.joy.get_ball(i))

        self.button = []
        for i in range(self.numbuttons):
            self.button.append(self.joy.get_button(i))

        self.hat = []
        for i in range(self.numhats):
            self.hat.append(self.joy.get_hat(i))


class World(object):
    def __init__(self, carla_world, hud, adas, actor_filter):
        self.world = carla_world
        self.hud = hud
        self.adas = adas
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.recording_enabled = False
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            blueprint.set_attribute('color', '135,255,240')
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[0] if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)
        self.adas.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)
        self.adas.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, client, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self.client = client
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
            self._lights = carla.VehicleLightState.NONE
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = joystick_handler(0)

        self.name = eg.enterbox(msg='Nombre del conductor:',
                                title='Conductor',
                                default='anonimo', strip=True,
                                image=None).replace(" ", "_")


        '''
        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))
        '''
        self._steer_idx = 0
        self._throttle_idx = 5
        self._brake_idx = 4
        self._reverse_idx = 3
        self._handbrake_idx = 1

        self._joystick.axis[self._brake_idx] = -1
        self._joystick.axis[self._throttle_idx] = -1

        self._light_beam_left = False
        self._light_beam_right = False


        #4 left        5 rigth
        #6  7
        # 

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                self._joystick.button[event.button] = 1
                if event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 0:
                    world.camera_manager.next_sensor()
                elif event.button == 6:
                    world.hud.notification("Boton 6")
                    world.camera_manager.toggle_camera()
                elif event.button == 4:
                    if(self._lights != carla.VehicleLightState.LeftBlinker):
                        self._lights = carla.VehicleLightState.NONE
                        self._lights = carla.VehicleLightState.LeftBlinker
                        world.hud.notification("Intermitente Izqd ON")
                    else:
                        self._lights = carla.VehicleLightState.NONE
                        world.hud.notification("Intermitente Izqd OFF")
                elif event.button == 5:
                    if(self._lights != carla.VehicleLightState.RightBlinker):
                        self._lights = carla.VehicleLightState.NONE
                        self._lights = carla.VehicleLightState.RightBlinker
                        world.hud.notification("Intermitente Dcho ON")
                    else:
                        self._lights = carla.VehicleLightState.NONE
                        world.hud.notification("Intermitente Dcho OFF")
                elif event.button == 7:
                    if (world.recording_enabled):
                        self.client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        if not os.path.exists(os.path.join(HOME, self.name)):
                            os.makedirs(os.path.join(HOME, self.name))
                        self.client.start_recorder(os.path.join(HOME, self.name,str("recording_"+str(datetime.datetime.now().strftime("%m_%d_%YT%H_%M_%S"))+".rec")))
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")

            elif event.type == JOYAXISMOTION:
                self._joystick.axis[event.axis] = event.value
            elif event.type == JOYBALLMOTION:
                self._joystick.ball[event.ball] = event.rel
            elif event.type == JOYHATMOTION:
                self._joystick.hat[event.hat] = event.value
            elif event.type == JOYBUTTONUP:
                self._joystick.button[event.button] = 0

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())

            world.player.apply_control(self._control)
            world.player.set_light_state(carla.VehicleLightState(self._lights))


    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        jsInputs = self._joystick.axis
        jsButtons = self._joystick.button

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = jsInputs[self._steer_idx]*pow(abs(jsInputs[self._steer_idx]), 2)
        steerCmd = 0 if abs(steerCmd)<0.02 else steerCmd/2
        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        throttleCmd = 1 - throttleCmd
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        brakeCmd = 1 - brakeCmd
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

class ADAS(object):
    def __init__(self, width, height, vehicles_list):
        self.dim = (width, height)
        self.frame = 0
        self.vehicles_list_ids = vehicles_list
        self.vehicles_list = None
        self.player = None
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        if not self._show_info:
            return
        if self.vehicles_list is None:
            self.vehicles_list = world.world.get_actors(self.vehicles_list_ids)

        self.player = world.player
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        self.lidarValue = world.camera_manager.lidarValue

    def render(self, display):
        adas_surface = pygame.Surface((400,200))
        adas_surface.fill((128,128,128))
        if self.lidarValue is not None:
            for position in self.lidarValue:
                pass
                #print(position)
        
        pygame.draw.circle(adas_surface, (255, 0, 0, 1), (100,100), 40)
        pygame.draw.circle(adas_surface, (255, 255, 0, 0.5), (200,100), 40)
        pygame.draw.circle(adas_surface, (0, 255, 0, 0.5), (300,100), 40)
        display.blit(adas_surface, (self.dim[0]-400, self.dim[1]-200))

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % "",
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = sorted([(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id])[:5]
            for d, vehicle in vehicles:
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, int(self.dim[1]*0.4)))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, self.dim[1]-int(self.dim[1]*0.4)))
            v_offset = self.dim[1]-int(self.dim[1]*0.4) +4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False

        bound_x = 0.5 + parent_actor.bounding_box.extent.x
        bound_y = 0.5 + parent_actor.bounding_box.extent.y
        bound_z = 0.5 + parent_actor.bounding_box.extent.z

        self.sensors = [
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

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                bp.set_attribute('fov', str(120))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            
            for attr_name, attr_value in item[3].items():
                bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        
        self.images = {}
        self.images['left'] = None
        self.images['right'] = None

        def showImage(weak_self, frame, image):
            self = weak_self()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (int(image.height), int(image.width), 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            array = cv2.flip(array, 1)
            self.images[frame] = array.swapaxes(0, 1)

        def showImage_izquierda(weak_self, frame, image):
            self = weak_self()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (int(image.height), int(image.width), 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            #array = cv2.flip(array, 1)
            self.images[frame] = array.swapaxes(0, 1)

        self._camera_transforms = [
                (carla.Transform(carla.Location(x=bound_x/15, y=-0.4, z=1.15), carla.Rotation()), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=+0.6, y=-bound_y*0.59, z=1.03), carla.Rotation(yaw=180+15)), carla.AttachmentType.Rigid),
                #(carla.Transform(carla.Location(x=+0.6, y=-bound_y*1.59, z=1.03), carla.Rotation(yaw=-20)), carla.AttachmentType.Rigid),
                
                (carla.Transform(carla.Location(x=+0.6, y=bound_y*0.59, z=1.03), carla.Rotation(yaw=180-15)), carla.AttachmentType.Rigid),
                (carla.Transform(carla.Location(x=-bound_x, z=1.1), carla.Rotation(yaw=180)), carla.AttachmentType.Rigid),

                (carla.Transform(carla.Location(x=bound_x/15, y=-0.4, z=1.15), carla.Rotation(yaw=270+30)), carla.AttachmentType.Rigid), ##izquierda
               ]
                             
        weak_self = weakref.ref(self)

        sensor_left_bp = bp_library.find('sensor.camera.rgb')
        sensor_left_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_left_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_left_bp.set_attribute('fov', str(120))

        sensor_r_bp = bp_library.find('sensor.camera.rgb')
        sensor_r_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_r_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_r_bp.set_attribute('fov', str(30))


        sensor_back_bp = bp_library.find('sensor.camera.rgb')
        sensor_back_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_back_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.1)))
        sensor_back_bp.set_attribute('fov', str(90))

        
        #new code

        sensor_izquierda_bp = bp_library.find('sensor.camera.rgb')
        sensor_izquierda_bp.set_attribute('image_size_x', str(int(hud.dim[0]*0.2)))
        sensor_izquierda_bp.set_attribute('image_size_y', str(int(hud.dim[1]*0.2)))
        sensor_izquierda_bp.set_attribute('fov', str(80))


        ## end new code

        self.sensor_back = parent_actor.get_world().spawn_actor(
                sensor_back_bp,
                self._camera_transforms[3][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_back.listen(lambda image: showImage(weak_self, "back",image))

        self.sensor_left = parent_actor.get_world().spawn_actor(
                sensor_left_bp,
                self._camera_transforms[1][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_left.listen(lambda image: showImage(weak_self, "left",image))

        self.sensor_right = parent_actor.get_world().spawn_actor(
                sensor_left_bp,
                self._camera_transforms[2][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_right.listen(lambda image: showImage(weak_self, "right",image))
        
        #new code

        self.sensor_izquierda = parent_actor.get_world().spawn_actor(
                sensor_izquierda_bp,
                self._camera_transforms[4][0],
                attach_to=parent_actor,
                attachment_type=self._camera_transforms[0][1])

        self.sensor_izquierda.listen(lambda image: showImage_izquierda(weak_self, "izquierda",image))


        #end new code

        self.transform_index = 0

        def lidarControl(weak_self, value):
            self = weak_self()
            self.lidarValue = value

        self.lidarValue = None
        self.index = None
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(32))
        lidar_bp.set_attribute('points_per_second',str(90000))
        lidar_bp.set_attribute('rotation_frequency',str(40))
        lidar_bp.set_attribute('range',str(20))
        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        self.lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=parent_actor)
        self.lidar_sen.listen(lambda point_cloud: lidarControl(weak_self, point_cloud))


    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(pygame.surfarray.make_surface(self.surface), (0, 0))
        if self.images['left'] is not None:
            display.blit(pygame.surfarray.make_surface(self.images['left']), (0, 0))

        if self.images['right'] is not None:
            display.blit(pygame.surfarray.make_surface(self.images['right']), (self.hud.dim[0]-(self.hud.dim[0]*0.2), 0))


        if self.images['back'] is not None:
            display.blit(pygame.surfarray.make_surface(self.images['back']), (int(self.hud.dim[0]/2)-(self.hud.dim[0]*0.1), 0))

        if self.images['izquierda'] is not None:
            display.blit(pygame.surfarray.make_surface(self.images['izquierda']), (0, 400))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data) # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = lidar_img
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = array.swapaxes(0, 1)
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))
    
    pygame.font.init()
    world = None
    client = None
    vehicles_list = []
    walkers_list = []
    all_id = []
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        adas = ADAS(args.width, args.height, vehicles_list)
        world = World(client.get_world(), hud, adas, args.filter)
        client.get_world().unload_map_layer(carla.MapLayer.All)
        client.get_world().load_map_layer(carla.MapLayer.Ground)
        controller = DualControl(world, client, args.autopilot)
        

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        
        original_settings = client.get_world().get_settings()
        settings = client.get_world().get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.03
        client.get_world().apply_settings(settings)

        spectator = client.get_world().get_spectator()
        
        blueprints = get_actor_blueprints(client.get_world(), args.filterv, args.generationv)
        blueprintsWalkers = get_actor_blueprints(client.get_world(), args.filterw, args.generationw)

        blueprints = [x for x in blueprints if not x.id.endswith('model3')]

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = client.get_world().get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        #######################################################################################################################################
        #                                                 DIBUJAR LOS ÍNDICES DE LOS SPAWN POINTS                                             #
        #######################################################################################################################################

        for i, spawn_point in enumerate(spawn_points):
            client.get_world().debug.draw_string(spawn_point.location, str(i), life_time=30)

        #######################################################################################################################################
        #                                           GENERACIÓN DE RUTAS PARA VEHÍCULOS EN LA SIMULACIÓN                                       #
        #######################################################################################################################################

        # Ruta 1
        spawn_point_1 = spawn_points[163]
        # Crear ruta 1 con los spawn points
        route_1_indices = [210, 232, 230, 212, 38, 165, 35, 154]
        route_1 = []
        for ind in route_1_indices:
            route_1.append(spawn_points[ind].location)

        # Ruta 2
        spawn_point_2 = spawn_points[146]
        # Crear ruta 1 con los spawn points
        route_2_indices = [120, 247, 136, 228, 85, 260, 58]
        route_2 = []
        for ind in route_2_indices:
            route_2.append(spawn_points[ind].location)

        # Imprimo en el mapa para ver la ruta
        client.get_world().debug.draw_string(spawn_point_1.location, 'Spawn Point 1', life_time=30, color=carla.Color(0,0,255))
        client.get_world().debug.draw_string(spawn_point_2.location, 'Spawn Point 2', life_time=30, color=carla.Color(0,255,0))

        for ind in route_1_indices:
            spawn_points[ind].location
            client.get_world().debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(0,0,255))

        for ind in route_2_indices:
            spawn_points[ind].location
            client.get_world().debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(0,255,0))

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        '''
        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            light_state = vls.NONE
            
            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                .then(SetVehicleLightState(FutureActor, light_state)))

        for response in client.apply_batch_sync(batch, True):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # -------------
        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = client.get_world().get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = client.get_world().get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = client.get_world().get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
     
        client.get_world().tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        client.get_world().set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(client.get_world().get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
        
        player = None
        for actor in client.get_world().get_actors().filter('vehicle.tesla.model3'):
            if actor.attributes.get('role_name') == 'hero':
                player = actor
        
        '''
        clock = pygame.time.Clock()

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(-30.0)
        '''
        # Coche extra de Borja
        extra_v = client.get_world().try_spawn_actor(random.choice(blueprints), spawn_points[219])
        if extra_v:     # Si el vehículo extra_v spawnea correctamente
            extra_v.set_autopilot(True)

            # Parámetros de autopilot del vehículo extra_v
            traffic_manager.update_vehicle_lights(extra_v, True)
            traffic_manager.random_left_lanechange_percentage(extra_v, False)
            traffic_manager.random_right_lanechange_percentage(extra_v, False)
            traffic_manager.auto_lane_change(extra_v, False)
        vehicles_list.append(extra_v)
        '''
        # Poner delay para crear un espacio entre momentos de spawn
        spawn_delay = 200
        counter = spawn_delay

        # Poner vehículos máximos
        max_vehicles = 30
        # Alternar entre spawn points
        alt = False

        while True:
            clock.tick_busy_loop(60)
            client.get_world().tick()
            if controller.parse_events(world, clock):
                return
            world.tick(clock)

            n_vehicles = len(vehicles_list)
            vehicle_bp = random.choice(blueprints)

            # Spawnear vehículo después del delay
            if counter == spawn_delay and n_vehicles < max_vehicles:
                # Alternar spawn points
                if alt:
                    vehicle = client.get_world().try_spawn_actor(vehicle_bp, spawn_point_1)
                else:
                    vehicle = client.get_world().try_spawn_actor(vehicle_bp, spawn_point_2)

                if vehicle:     # Si el vehículo spawnea correctamente
                    vehicle.set_autopilot(True)

                    # Parámetros de autopilot del vehículo
                    traffic_manager.update_vehicle_lights(vehicle, True)
                    traffic_manager.random_left_lanechange_percentage(vehicle, False)
                    traffic_manager.random_right_lanechange_percentage(vehicle, False)
                    traffic_manager.auto_lane_change(vehicle, False)

                    # Alternar rutas
                    if alt:
                        traffic_manager.set_path(vehicle, route_1)
                        alt = False
                    else:
                        traffic_manager.set_path(vehicle, route_2)
                        alt = True

                    vehicles_list.append(vehicle)
                    vehicle = None

                counter -= 1
            elif counter > 0:
                counter -= 1
            elif counter == 0:
                counter = spawn_delay
            elif n_vehicles >= max_vehicles:
                vehicle = None

            world.render(display)
            pygame.display.flip()

    finally:
        if client!=None:
            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
            '''
            # stop walker controllers (list is [controller, actor, controller, actor ...])
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()

            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
            '''
            client.reload_world(True)
        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    args = argparser.parse_args()

    args.width, args.height = int(monitor.width*0.9), int(monitor.height*0.9)

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
