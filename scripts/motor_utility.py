#!/usr/bin/env python
"""Utility to test ROSBot motors."""

import argparse
import me416_utilities as mu
import time

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
SPEED_FACTOR = 0.95


def apply_action(L_motor, R_motor, direction, side, is_recursive=True, duration=2):
    """Apply the specified action to the specified side"""
    if direction == 'forward':
        speed = 1
    elif direction == 'backward':
        speed = -1
    elif direction == 'stop':
        speed = 0

    if side == 'left' or side == 'both':
        L_motor.set_speed(speed)

    if side == 'right' or side == 'both':
        R_motor.set_speed(speed)

    if is_recursive:
        time.sleep(duration)
        apply_action(L_motor, R_motor, 'stop', 'both', is_recursive=False)


def main(args):
    """Create motor objects, then apply specified commands."""

    L_motor = mu.MotorSpeedLeft(SPEED_FACTOR)
    R_motor = mu.MotorSpeedRight()

    apply_action(L_motor, R_motor, args.direction, args.side, args.duration)


if __name__ == "__main__":
    example_text="""example usage: ./motor_utility.py forward --side left --duration 5"""
    parser = argparse.ArgumentParser(
        description='Utility to test ROSBot motors.',
        epilog=example_text,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('direction')
    parser.add_argument('--side', type=str, default='both', help='Side to run, can be "left", "right", or "both"')
    parser.add_argument('--duration', type=int, default=2, help='Duration of the motor activation')
    args = parser.parse_args()
    main(args)
