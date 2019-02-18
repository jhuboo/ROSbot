#!/usr/bin/env python
"""Utility to test ROSBot motors."""
import argparse
import me416_utilities as mu
import time

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
SPEED_FACTOR = 0.95


def apply_action(L_motor, R_motor, direction, side, seconds,
                 is_recursive=True):
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
        time.sleep(float(seconds))
        apply_action(
            L_motor, R_motor, 'stop', 'both', seconds, is_recursive=False)


def main(args):
    """Create motor objects, then apply specified commands."""

    L_motor = mu.MotorSpeedLeft(SPEED_FACTOR)
    R_motor = mu.MotorSpeedRight()

    apply_action(L_motor, R_motor, args.direction, args.side, args.seconds)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Utility to test ROSBot motors.')
    parser.add_argument('direction')
    parser.add_argument('side', nargs='?', default='both')
    parser.add_argument('seconds', nargs='?', default=2)
    args = parser.parse_args()
    main(args)
