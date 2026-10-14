# Use one settings class so the GUI and robot interface share the same limits and timing
# The Arduino uses the same drive limit, baud rate, and timeout in its own program
class MotorSettings:
    # These are signed motor drive values from the Pololu library, not angles or RPM
    MAX_COMMAND = 400

    # Send the left and right targets together on this ROS topic
    TOPIC = '/motor_cmd'

    # Match the baud rate configured by Serial begin in the Arduino program
    SERIAL_BAUD = 115200
    DEFAULT_PORT = '/dev/ttyACM0'

    # Send fresh commands every 100 ms and remove drive after a 500 ms command gap
    COMMAND_PERIOD_SECONDS = 0.1
    COMMAND_TIMEOUT_SECONDS = 0.5
