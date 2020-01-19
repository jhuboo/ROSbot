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
    encoder = mu.QuadEncoder(3, 5, 0.1)
    try:
        encoder.start()
        for i in range(1000):
            print('Velocity is: ' + str(encoder.getVelocity()))
            time.sleep(0.1)
        encoder.stop()
        encoder.join()
    except ServiceExit:
        # Shut down signal received
        encoder.stop()
        encoder.join()
