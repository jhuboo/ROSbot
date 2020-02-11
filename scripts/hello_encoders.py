#!/usr/bin/env python
"""
Test script to read encoders
"""

import me416_utilities as mu
import time
import signal


class ServiceExit(Exception):
    """
    Custom exception to handle sigint
    """
    pass


def exit_gracefully(signum, frame):
    raise ServiceExit


if __name__ == "__main__":
    signal.signal(signal.SIGINT, exit_gracefully)
    signal.signal(signal.SIGTERM, exit_gracefully)
    encoderRight = mu.QuadEncoderRight()
    encoderLeft = mu.QuadEncoderLeft()
    try:
        encoderRight.start()
        encoderLeft.start()
        for i in range(1000):
            print('Right Velocity: ' + str(encoderRight.getVelocity()) +
                  ' Left Velocity: ' + str(encoderLeft.getVelocity()))
            time.sleep(0.1)
    except ServiceExit:
        # Shut down signal received
        print('Stopping')
    encoderRight.stop()
    encoderLeft.stop()
    encoderRight.join()
    encoderLeft.join()
