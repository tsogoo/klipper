# Code for handling the kinematics of scara robots
#
# Copyright (C) 2022  Tsogtgerel Amar <ts.r.madrid@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper

class ScaraKinematics:
    def __init__(self, toolhead, config):

        scara_config = config.getsection('scara')
        self.shoulder_length = scara_config.getfloat(
            'shoulder_length', minval=0.)
        self.arm_length = scara_config.getfloat('arm_length', minval=0.)
        self.gantry_shoulder = scara_config.getfloat(
            'gantry_shoulder', minval=0.)
        self.gantry_arm = scara_config.getfloat('gantry_arm', minval=0.)
        self.shoulder_pulley_ratio = scara_config.getfloat(
            'shoulder_pulley_ratio', above=0.)
        self.arm_pulley_ratio = scara_config.getfloat(
            'arm_pulley_ratio', above=0.)
        self.arm_relative = scara_config.boolean('arm_relative', None)

        stepper_shoulder = stepper.PrinterStepper(
            config.getsection('stepper_s'), units_in_radians=True)
        stepper_arm = stepper.PrinterStepper(config.getsection('stepper_a'),
                                            units_in_radians=True)
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        
        stepper_shoulder.setup_itersolve(
            'scara_stepper_alloc', self.shoulder_length, self.arm_length,
            self.gantry_shoulder, self.gantry_arm, b's')
        stepper_arm.setup_itersolve(
            'scara_stepper_alloc', self.shoulder_length, self.arm_length,
            self.gantry_shoulder, self.gantry_arm, b'a')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        
        self.rails = [rail_z]
        self.steppers = [stepper_shoulder, stepper_arm] + \
            [ s for r in self.rails for s in r.get_steppers() ]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        shoulder_angle = stepper_positions[self.steppers[0].get_name()] * \
            self.shoulder_pulley_ratio
        arm_angle = stepper_positions[self.steppers[1].get_name()] * \
            self.arm_pulley_ratio
        if self.arm_relative:
            arm_angle -= shoulder_angle
        z_pos = stepper_positions[self.rails[0].get_name()]
        return [
            # x axis
            self.shoulder_length * math.sin(shoulder_angle) + \
        (self.arm_length * math.sin(arm_angle) - \
        (self.gantry_shoulder * math.sin(arm_angle) - \
            self.gantry_arm * math.cos(arm_angle))),
            # y axis
            self.shoulder_length * math.cos(shoulder_angle) + \
        (self.arm_length * math.cos(arm_angle) + \
        (self.gantry_shoulder * math.cos(arm_angle) + \
            self.gantry_arm * math.sin(arm_angle))),
            # z axis
            z_pos
        ]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return ScaraKinematics(toolhead, config)
