from .parse_motor_command import parse_motor_command


# Build one command containing both targets so each message updates the complete pair
# Reuse the parser here so code cannot publish a command that the robot would reject
def format_motor_command(left, right):
    command = f'M {left} {right}'
    parse_motor_command(command)
    return command
