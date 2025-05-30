#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// ---------------------
// Robot Configuration
// ---------------------
const float wheelDiameterCm       = 6.3;
const float wheelBaseCm           = 15.5;
const float distPerPulseCm        = 0.518519;

const int   encoderPulsesPerRev   = 20;
const float wheelCircumferenceCm  = PI * wheelDiameterCm;

// ---------------------
// Odometry State
// ---------------------
volatile long encoderCountLeft  = 0;
volatile long encoderCountRight = 0;
float xPosCm      = 0;
float yPosCm      = 0;
float headingDeg  = 0;
bool  headingInitialized = false;
float headingOffsetDeg   = 0;
volatile bool countEncoders = true;  // flag to enable/disable counting

// ---------------------
// Encoder Pins
// ---------------------
#define ENCODER_LEFT_PIN  34
#define ENCODER_RIGHT_PIN 35

// ---------------------
// Motor Pins & Speed
// ---------------------
const int enA = 4, in1 = 18, in2 = 19;
const int enB = 5, in3 = 25, in4 = 26;
const int motorSpeedPWM = 130;

// ---------------------
// Movement Flags
// ---------------------
int movementDir = 0;

// ---------------------
// Compass (GY-271 / HMC5883L)
// ---------------------
Adafruit_HMC5883_Unified hmc5883 = Adafruit_HMC5883_Unified(12345);

// ---------------------
// ISR Routines
// ---------------------
void IRAM_ATTR onEncoderLeft() {
  if (countEncoders) encoderCountLeft++;
}
void IRAM_ATTR onEncoderRight() {
  if (countEncoders) encoderCountRight++;
}

// ---------------------
// WebSocket & WiFi
// ---------------------
WebSocketsClient webSocket;
const char* ssid     = "TUFa15";
const char* password = "123456789";

// ---------------------
// Send WebSocket Event
// ---------------------
void sendEvent(const char* eventType) {
  StaticJsonDocument<128> doc;
  doc["event"] = eventType;
  doc["payload"] = JsonObject(); // empty payload
  String msg;
  serializeJson(doc, msg);
  webSocket.sendTXT(msg);
}

// ---------------------
// Compass Heading
// ---------------------
void updateHeading() {
  sensors_event_t event;
  hmc5883.getEvent(&event);
  float raw = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
  if (raw < 0) raw += 360;
  if (!headingInitialized) {
    headingInitialized = true;
    headingOffsetDeg   = raw;
    headingDeg         = 0;
  } else {
    headingDeg = raw - headingOffsetDeg;
    if (headingDeg < 0) headingDeg += 360;
  }
}

// ---------------------
// Position Update
// ---------------------
void updatePosition() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < 100) return;
  lastMs = now;

  noInterrupts();
    long leftP  = encoderCountLeft;
    long rightP = encoderCountRight;
    encoderCountLeft  = 0;
    encoderCountRight = 0;
  interrupts();

  float dL = leftP  * distPerPulseCm;
  float dR = rightP * distPerPulseCm;
  float dA = (dL + dR) / 2.0;

  int dir = movementDir;
  if (dir == 0 && dA > 0) dir = +1;

  float theta = headingDeg * PI / 180.0;
  xPosCm += dir * dA * cos(theta);
  yPosCm += dir * dA * sin(theta);

  Serial.printf(
    "Pulses L:%4ld R:%4ld  Dist:%.2fcm  Dir:%2d  H:%.1f°  →  X:%.2fcm  Y:%.2fcm\n",
    leftP, rightP, dA, dir, headingDeg, xPosCm, yPosCm
  );
}

// ---------------------
// Motor Control
// ---------------------
void moveForward() {
  countEncoders = true;
  movementDir = +1;
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedPWM);
  analogWrite(enB, motorSpeedPWM);
  sendEvent("forward");
}

void moveBackward() {
  countEncoders = true;
  movementDir = -1;
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedPWM);
  analogWrite(enB, motorSpeedPWM);
  sendEvent("reverse");
}

void turnRight() {
  countEncoders = false;
  movementDir = 0;
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedPWM);
  analogWrite(enB, motorSpeedPWM);
  sendEvent("turning");
}

void turnLeft() {
  countEncoders = false;
  movementDir = 0;
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedPWM);
  analogWrite(enB, motorSpeedPWM);
  sendEvent("turning");
}

void stopMotor() {
  countEncoders = true;
  movementDir = 0;
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  sendEvent("stop");
}

// ---------------------
// Send Odometry Data
// ---------------------
void sendData() {
  StaticJsonDocument<256> doc;
  doc["event"] = "update";
  JsonObject payload = doc.createNestedObject("payload");
  payload["heading"] = headingDeg;
  payload["X"] = xPosCm;
  payload["Y"] = -yPosCm;  // <-- Flipped Y sign here
  String out;
  serializeJson(doc, out);
  webSocket.sendTXT(out);
}

// ---------------------
// Handle Incoming WebSocket Events
// ---------------------
void webSocketEvent(WStype_t type, uint8_t *msg, size_t len) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    if (!deserializeJson(doc, msg)) {
      const char* evt = doc["event"];
      JsonObject drv = doc["payload"];

      if (strcmp(evt, "command") == 0) {
        if      (drv["forward"])  moveForward();
        else if (drv["reverse"])  moveBackward();
        else if (drv["left"])     turnLeft();
        else if (drv["right"])    turnRight();
        else if (drv["stop"])     stopMotor();
      }
      else if (strcmp(evt, "reset") == 0) {
        xPosCm = yPosCm = 0;
        encoderCountLeft = encoderCountRight = 0;
        headingDeg = 0;
        headingInitialized = false;
        sendEvent("reset_done");
      }
    }
  }
}

// ---------------------
// Setup
// ---------------------
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  webSocket.beginSSL("robot-to-3d-server.onrender.com", 443, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  if (!hmc5883.begin()) {
    Serial.println("Compass not found");
    while (1);
  }

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN),  onEncoderLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onEncoderRight, RISING);

  stopMotor();
}

// ---------------------
// Main Loop
// ---------------------
void loop() {
  webSocket.loop();
  updateHeading();
  updatePosition();

  static unsigned long lastSent = millis();
  if (millis() - lastSent > 1000) {
    sendData();
    lastSent = millis();
  }
}
