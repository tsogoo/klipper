// scara kinematics stepper pulse time generation
//
// Copyright (C) 2022  Tsogtgerel Amar <ts.r.madrid@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct scara_stepper {
    struct stepper_kinematics sk;
    double shoulder_length; double arm_length;
    double gantry_shoulder; double gantry_arm;
};
static double
scara_stepper_shoulder_angle_calc_position(struct stepper_kinematics *sk
                             , struct move *m
                             , double move_time)
{
    struct scara_stepper *ss = container_of(sk, struct scara_stepper
                             , sk);
    struct coord c = move_get_coord(m, move_time);
    
    return sqrt((ss->shoulder_length - c.x) * (ss->shoulder_length - c.x) + c.y*c.y);
}

static double
scara_stepper_arm_angle_calc_position(struct stepper_kinematics *sk
                             , struct move *m
                             , double move_time)
{
    struct scara_stepper *ss = container_of(sk, struct scara_stepper
                             , sk);
    struct coord c = move_get_coord(m, move_time);
    return sqrt(c.y * c.y +  c.x * c.x);
}

struct stepper_kinematics * __visible
scara_stepper_alloc(
        double shoulder_length, double arm_length, 
        double gantry_shoulder, double gantry_arm
        , char side)
{
    struct scara_stepper *ss = malloc(sizeof(*ss));
    memset(ss, 0, sizeof(*ss));
    if (side == 's') {
        ss->sk.calc_position_cb = scara_stepper_shoulder_angle_calc_position;
    } else if (side == 'a') {
        ss->sk.calc_position_cb = scara_stepper_arm_angle_calc_position;
    }
    ss->sk.active_flags = AF_X | AF_Y;
    ss->shoulder_length = shoulder_length;
    ss->arm_length = shoulder_length;
    ss->gantry_shoulder = gantry_shoulder;
    ss->gantry_arm = gantry_arm;
    
    return &ss->sk;
}
