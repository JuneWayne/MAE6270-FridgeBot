import re

from .motor_settings import MotorSettings


# Read exactly two signed integer targets from a ROS String such as M 100 -100
# Reject extra text and out-of-range values before anything reaches the Arduino
def parse_motor_command(text):
    match = re.fullmatch(r'M ([+-]?[0-9]{1,3}) ([+-]?[0-9]{1,3})', text.strip())
    if match is None:
        raise ValueError('Use M <left> <right>')

    targets = tuple(map(int, match.groups()))
    if any(abs(value) > MotorSettings.MAX_COMMAND for value in targets):
        raise ValueError('Motor commands must be between -400 and 400')
    return targets
