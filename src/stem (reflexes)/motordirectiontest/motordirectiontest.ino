#include <esp_task_wdt.h>

// =================== PIN DEFINITIONS ===================
// Ultrasonic
const int trigPin = 25;
const int echoPin = 26;

// UART to Jetson (Serial2)
#define RXD2 32  // ESP32 RX  <- Jetson TX
#define TXD2 33  // ESP32 TX  -> Jetson RX

// Motors (your PCB pins)
#define AIN1  23
#define AIN2  22
#define PWMA  21

#define BIN1  19
#define BIN2  18
#define PWMB  17

#define STBY  16

// =================== CONFIG ===================
const int THRESH_CM = 25;                  // stop if object closer than this
const unsigned long US_PERIOD_MS = 50;     // ultrasonic update rate (~20 Hz)

// Heartbeat and watchdog timeouts
const unsigned long HEARTBEAT_TIMEOUT_MS = 500;   // 0.5 s -> emergency stop
const int WATCHDOG_TIMEOUT_S = 1;                 // 1.0 s -> MCU reset

// If you want PWM speed control later, replace digitalWrite(PWMx,HIGH) with analogWrite(PWMx,speed)
const bool USE_FULL_POWER = true;          
const uint8_t PWM_SPEED = 180;             

// UART line buffer
const size_t UART_BUF_SIZE = 32;
char uartBuf[UART_BUF_SIZE];
size_t uartIdx = 0;

// =================== STATE ===================
enum MotionCmd { CMD_STOP, CMD_FWD, CMD_LEFT, CMD_RIGHT };
volatile MotionCmd desiredCmd = CMD_STOP;
volatile bool safetyStop = false;
volatile bool heartbeatStop = true;   // safe by default until heartbeat is received

unsigned long lastHeartbeatMs = 0;

// =================== MOTOR CONTROL ===================
void motors_stop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
}

void motors_forward(uint8_t speed) {
  // Left motor (A) forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Right motor (B) forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  if (USE_FULL_POWER) {
    digitalWrite(PWMA, HIGH);
    digitalWrite(PWMB, HIGH);
  } else {
    analogWrite(PWMA, speed);
    analogWrite(PWMB, speed);
  }
}

void motors_left(uint8_t speed) {
  // Left motor (A) OFF
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  // Right motor (B) forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  if (USE_FULL_POWER) {
    digitalWrite(PWMA, LOW);
    digitalWrite(PWMB, HIGH);
  } else {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, speed);
  }
}

void motors_right(uint8_t speed) {
  // Left motor (A) forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  // Right motor (B) OFF
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  if (USE_FULL_POWER) {
    digitalWrite(PWMA, HIGH);
    digitalWrite(PWMB, LOW);
  } else {
    analogWrite(PWMA, speed);
    analogWrite(PWMB, 0);
  }
}

void applyOutputs() {
  if (safetyStop || heartbeatStop) {
    motors_stop();
    return;
  }

  switch (desiredCmd) {
    case CMD_FWD:
      motors_forward(PWM_SPEED);
      break;

    case CMD_LEFT:
      motors_left(PWM_SPEED);
      break;

    case CMD_RIGHT:
      motors_right(PWM_SPEED);
      break;

    case CMD_STOP:
    default:
      motors_stop();
      break;
  }
}

// =================== ULTRASONIC ===================
float readDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999.0;

  float distance = (duration * 0.0343f) / 2.0f;
  return distance;
}

// =================== UART COMMAND PARSING ===================
void handleJetsonLine(const char* lineRaw) {
  String line = String(lineRaw);
  line.trim();
  if (line.length() == 0) return;

  Serial.print("JETSON DATA: ");
  Serial.println(line);

  // HEARTBEAT separate from motion commands
  if (line == "H" || line == "h" || line == "HB" || line == "hb") {
    lastHeartbeatMs = millis();
    heartbeatStop = false;
    Serial.println("HEARTBEAT -> OK");
  }
  else if (line == "g") {
    desiredCmd = CMD_FWD;
    Serial.println("CMD -> FORWARD");
  }
  else if (line == "l") {
    desiredCmd = CMD_LEFT;
    Serial.println("CMD -> LEFT");
  }
  else if (line == "r") {
    desiredCmd = CMD_RIGHT;
    Serial.println("CMD -> RIGHT");
  }
  else if (line == "s") {
    desiredCmd = CMD_STOP;
    Serial.println("CMD -> STOP");
  }
  else {
    Serial.println("CMD -> UNKNOWN");
  }
}

// =================== NON-BLOCKING UART PARSER ===================
void processJetsonUART() {
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();

    if (c == '\n' || c == '\r') {
      if (uartIdx > 0) {
        uartBuf[uartIdx] = '\0';
        handleJetsonLine(uartBuf);
        uartIdx = 0;
      }
    }
    else {
      if (uartIdx < UART_BUF_SIZE - 1) {
        uartBuf[uartIdx++] = c;
      } else {
        uartIdx = 0;
        Serial.println("UART BUFFER OVERFLOW -> line dropped");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  desiredCmd = CMD_STOP;
  heartbeatStop = true;
  safetyStop = false;
  motors_stop();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  delay(500);
  Serial.println("Setup done");
}

void loop() {
  esp_task_wdt_reset();

  // 1) Non-blocking UART processing
  processJetsonUART();

  // 2) Heartbeat timeout check
  if ((millis() - lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS) {
    heartbeatStop = true;
  }

  // 3) Periodically update ultrasonic safety
  static unsigned long lastUS = 0;
  if (millis() - lastUS >= US_PERIOD_MS) {
    lastUS = millis();
    float dist = readDistanceCM();

    safetyStop = (dist < THRESH_CM);

    Serial.print("Distance(cm): ");
    Serial.print(dist);
    Serial.print(" | safetyStop: ");
    Serial.print(safetyStop ? "YES" : "NO");
    Serial.print(" | heartbeatStop: ");
    Serial.println(heartbeatStop ? "YES" : "NO");

  }

  // 4) Apply outputs with safety gating
  applyOutputs();
}