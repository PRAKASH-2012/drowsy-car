#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE = Pin 9, CSN = Pin 10
const byte address[6] = "CAR01";

// --- L298N Motor Pins ---
const int ENA = 5;  // Left Motors PWM
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 2;
const int IN4 = 3;
const int ENB = 6;  // Right Motors PWM

// --- Alert Indicators ---
const int buzzerPin = 4; // Buzzer Pin
const int ledPin    = A0; // Emergency LED Pin

int cruiseSpeed = 210; // Default motor speed (0-255), adjustable via SPD command
int currentSpeed = 0;
bool emergencyActive = false;

// Synchronous blinking strobe timer
unsigned long lastBlinkTime = 0;
bool blinkState = false;
const int BLINK_INTERVAL = 150; // Pulse every 150ms

// Hold-to-drive watchdog: stop if drive packets stop arriving
// (key released, USB unplugged, app killed, or packet loss)
unsigned long lastMotionTime = 0;
const unsigned long MOTION_TIMEOUT = 600; // ms

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  currentSpeed = 0;
}

void drive(bool in1, bool in2, bool in3, bool in4, int speed) {
  driveLR(in1, in2, in3, in4, speed, speed);
}

// Differential drive: independent left/right PWM (for gentle turns)
void driveLR(bool in1, bool in2, bool in3, bool in4, int speedL, int speedR) {
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);
  digitalWrite(IN1, in1 ? HIGH : LOW);
  digitalWrite(IN2, in2 ? HIGH : LOW);
  digitalWrite(IN3, in3 ? HIGH : LOW);
  digitalWrite(IN4, in4 ? HIGH : LOW);
  analogWrite(ENA, speedL); // Left motors
  analogWrite(ENB, speedR); // Right motors
  currentSpeed = max(speedL, speedR);
}

// Handle "CMD" or "CMD:speed" packets, e.g. "FWD", "FWD:210", "SPD:150"
void handleDriveCommand(char* cmd) {
  char motion[8] = "";
  int spd = cruiseSpeed;

  char* sep = strchr(cmd, ':');
  if (sep != NULL) {
    size_t n = sep - cmd;
    if (n >= sizeof(motion)) n = sizeof(motion) - 1;
    strncpy(motion, cmd, n);
    motion[n] = '\0';
    spd = constrain(atoi(sep + 1), 0, 255);
  } else {
    strncpy(motion, cmd, sizeof(motion) - 1);
    motion[sizeof(motion) - 1] = '\0';
  }

  lastMotionTime = millis(); // feed the hold-to-drive watchdog

  if (strcmp(motion, "SPD") == 0) {
    cruiseSpeed = spd;
    Serial.print("Cruise speed set to ");
    Serial.println(cruiseSpeed);
    return;
  }
  if (strcmp(motion, "FWD") == 0) {
    drive(true, false, true, false, spd);
  } else if (strcmp(motion, "REV") == 0) {
    drive(false, true, false, true, spd);
  } else if (strcmp(motion, "LFT") == 0) {
    drive(false, true, true, false, spd);
  } else if (strcmp(motion, "RGT") == 0) {
    drive(true, false, false, true, spd);
  } else if (strcmp(motion, "SLFT") == 0) {
    // Gentle left arc: left motors at ~35%, right at full
    driveLR(true, false, true, false, spd * 35 / 100, spd);
  } else if (strcmp(motion, "SRGT") == 0) {
    // Gentle right arc: right motors at ~35%, left at full
    driveLR(true, false, true, false, spd, spd * 35 / 100);
  } else if (strcmp(motion, "STOP") == 0) {
    stopMotors();
  }
}

// Controlled ramp down to HALF speed (car keeps creeping, alarm strobes)
void triggerEmergencySlowdown() {
  emergencyActive = true;

  int target = currentSpeed / 2;
  for (int spd = currentSpeed; spd > target; spd -= 25) {
    analogWrite(ENA, spd);
    analogWrite(ENB, spd);
    delay(80); // Deceleration curve delay
  }
  analogWrite(ENA, target);
  analogWrite(ENB, target);
  currentSpeed = target;

  Serial.print("DROWSY: slowed to half speed (");
  Serial.print(target);
  Serial.println("). Send RESET to stop.");
}

// Full stop after 10s+ of continuous sleep (alarm keeps strobing)
void triggerEmergencyStop() {
  emergencyActive = true;

  for (int spd = currentSpeed; spd >= 0; spd -= 25) {
    analogWrite(ENA, spd);
    analogWrite(ENB, spd);
    delay(80); // Deceleration curve delay
  }
  stopMotors();

  Serial.println("SLEEP 10s+: full stop. Send RESET to recover.");
}

void resetEmergencyState() {
  emergencyActive = false;
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);
  blinkState = false;
  stopMotors();
}

void setup() {
  Serial.begin(9600);

  // Motor pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Alarm pin modes
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  resetEmergencyState();

  // NRF24L01 setup
  if (!radio.begin()) {
    Serial.println("NRF24 Init Failed! Check 3.3V and wiring.");
    while (1);
  }

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  Serial.println("Receiver Ready: Motor, Buzzer & LED Armed.");
  Serial.println("Hold-to-drive + 600ms watchdog armed.");
}

void loop() {
  // --- Synchronized LED & Buzzer Strobe during Emergency ---
  if (emergencyActive) {
    if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = millis();
      blinkState = !blinkState;

      // LED and buzzer pulse together
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
      digitalWrite(buzzerPin, blinkState ? HIGH : LOW);
    }
  }

  // --- Hold-to-drive watchdog: auto-stop on lost link ---
  if (!emergencyActive && currentSpeed > 0 && millis() - lastMotionTime >= MOTION_TIMEOUT) {
    stopMotors();
    Serial.println("Watchdog: no drive packet, motors stopped.");
  }

  // --- Process Wireless RF Commands from Nano ---
  if (radio.available()) {
    char cmd[16] = "";
    radio.read(&cmd, sizeof(cmd));

    Serial.print("Received: ");
    Serial.println(cmd);

    // Emergency drowsiness detection: halve speed, keep alarm strobing
    if (strcmp(cmd, "DROWSY") == 0) {
      if (!emergencyActive) {
        triggerEmergencySlowdown();
      }
    }
    // Escalation: asleep 10s+ -> full stop (alarm keeps strobing)
    else if (strcmp(cmd, "SLEEP_STOP") == 0) {
      triggerEmergencyStop();
    }
    // Reset command (Press 'R' in Python)
    else if (strcmp(cmd, "RESET") == 0) {
  resetEmergencyState();

  // Self-test: 3 LED blinks + 1 buzzer chirp proves wiring is OK.
  // If you see NO blinks here, check LED polarity/resistor/wiring on A0.
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);
    }
    // Normal manual driving (locked out during emergency)
    else if (!emergencyActive) {
      handleDriveCommand(cmd);
    }
  }
}