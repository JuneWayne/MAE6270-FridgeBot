// This program runs on the Uno and turns USB serial commands into motor shield output
// Command format is M <left> <right> followed by a newline
#include <DualVNH5019MotorShield.h>
#include <stdlib.h>

DualVNH5019MotorShield motors;

// The library uses signed drive values from -400 to 400, not angles or measured RPM
const long MAX_COMMAND = 400;
const unsigned long TIMEOUT_MS = 500;

// M1 is the left side and M2 is the right side according to the Week 7 diagram
// Change either sign to -1 if that side rotates opposite to the intended direction
const int LEFT_DIRECTION = 1;
const int RIGHT_DIRECTION = 1;

// A fixed buffer avoids dynamic String allocation on the Uno
char inputLine[48];
byte inputLength = 0;
bool discardLine = false;
bool motorsActive = false;
bool faultLatched = false;
unsigned long lastCommandTime = 0;

// Zero removes drive and lets the wheels coast, rather than applying active braking
void stopMotors() {
  motors.setSpeeds(0, 0);
  motorsActive = false;
}

void rejectCommand() {
  stopMotors();
  Serial.println("ERR");
}

// Check the whole command before changing either motor output
void applyCommand(char *line) {
  if (line[0] != 'M' || line[1] != ' ') {
    rejectCommand();
    return;
  }

  char *end;
  char *leftStart = line + 2;
  long left = strtol(leftStart, &end, 10);
  if (end == leftStart || *end != ' ') {
    rejectCommand();
    return;
  }

  char *rightStart = end + 1;
  long right = strtol(rightStart, &end, 10);
  if (end == rightStart || *end != '\0') {
    rejectCommand();
    return;
  }

  if (left < -MAX_COMMAND || left > MAX_COMMAND ||
      right < -MAX_COMMAND || right > MAX_COMMAND || faultLatched) {
    rejectCommand();
    return;
  }

  motors.setSpeeds((int)left * LEFT_DIRECTION, (int)right * RIGHT_DIRECTION);
  lastCommandTime = millis();
  motorsActive = left != 0 || right != 0;
  Serial.println("OK");
}

void setup() {
  Serial.begin(115200);
  motors.init();
  stopMotors();
  Serial.println("READY");
}

void loop() {
  // Keep a shield fault active so commands cannot restart the motors before a board reset
  if (!faultLatched && (motors.getM1Fault() || motors.getM2Fault())) {
    faultLatched = true;
    stopMotors();
    Serial.println("FAULT");
  }

  // Limit serial work so the timeout check still runs when input arrives continuously
  for (byte count = 0; count < 64 && Serial.available() > 0; count++) {
    char character = Serial.read();
    if (character == '\r') continue;

    if (character == '\n') {
      if (!discardLine && inputLength > 0) {
        inputLine[inputLength] = '\0';
        applyCommand(inputLine);
      }
      inputLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (inputLength < sizeof(inputLine) - 1) {
        inputLine[inputLength++] = character;
      } else {
        // Ignore the rest of an oversized line so its tail cannot become a new command
        discardLine = true;
        rejectCommand();
      }
    }
  }

  // Unsigned time subtraction also works when the millis counter wraps around
  if (motorsActive && millis() - lastCommandTime >= TIMEOUT_MS) {
    stopMotors();
    Serial.println("TIMEOUT");
  }
}
