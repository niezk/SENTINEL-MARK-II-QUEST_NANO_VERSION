
/*
  Arduino Nano Line Follower Robot - 5 Sensor Version
  
  Pin Mapping for Arduino Nano:
  Buttons:
  UP    : D2
  DOWN  : D3
  +     : D4
  -     : D5
  Enter : D6
  Back  : D7
  
  I2C Display:
  SDA   : A4
  SCL   : A5
  
  Motors:
  DIRL  : D8
  PWML  : D9
  PWMR  : D10
  DIRR  : D11
  
  Sensors (5 sensors):
  LED   : D12
  S1    : A0 (leftmost)
  S2    : A1 (left)
  S3    : A2 (center)
  S4    : A3 (right)
  S5    : A6 (rightmost)
*/

#include <Wire.h>
#include <U8x8lib.h>
#include <EEPROM.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C lcd(U8X8_PIN_NONE);

#define MAXPLAN 4
#define NUM_SENSORS 5

struct Config {
  int8_t ACTION[MAXPLAN][100];
  int8_t SENSOR[MAXPLAN][100];
  int8_t RSPEED[MAXPLAN][100];
  int8_t LSPEED[MAXPLAN][100];
  int8_t COUNTERMODE[MAXPLAN][100];
  int16_t COUNTER[MAXPLAN][100];
  int8_t VA[MAXPLAN][100];
  int16_t COUNTA[MAXPLAN][100];
  int8_t COUNTERAMODE[MAXPLAN][100];
  int8_t VB[MAXPLAN][100];
  int16_t COUNTB[MAXPLAN][100];
  int8_t COUNTERBMODE[MAXPLAN][100];
  int8_t pid[MAXPLAN][100];
  int8_t FOLLOWMODE[MAXPLAN][100];
  int8_t JUMP[MAXPLAN][100];
  int8_t jumpP[MAXPLAN][100];
  int8_t jumpI[MAXPLAN][100];
  int8_t PLAN;
  int8_t I;
  int8_t STOP[MAXPLAN];
  int8_t CP[10];
  int8_t CPI;
  int8_t Va[10];
  uint8_t Ta[10];
  uint16_t sensorRef[NUM_SENSORS];
  uint8_t V;
  int8_t PID;
  uint8_t kp[3], kd[3];
  int8_t Vmin, Vmax;
  uint8_t Ts;
  int8_t LINE;
  int8_t sensitivity;
  int16_t Dly;
  bool stopAtFinish; // New parameter for finish line detection
} config;

// Button pins
#define BTN_OK 6
#define BTN_UP 2
#define BTN_DOWN 3
#define BTN_BACK 7
#define BTN_PLUS 4
#define BTN_MINUS 5

// Motor pins
#define MOTOR_L_DIR 8
#define MOTOR_L_PWM 9
#define MOTOR_R_PWM 10
#define MOTOR_R_DIR 11

// Sensor pins
#define LED_PIN 12
#define SENSOR_1 A0  // Leftmost
#define SENSOR_2 A1  // Left
#define SENSOR_3 A2  // Center
#define SENSOR_4 A3  // Right
#define SENSOR_5 A6  // Rightmost

// Sensor array
uint8_t sensorPins[NUM_SENSORS] = {SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5};
uint16_t sensorValues[NUM_SENSORS];
uint16_t bitSensor = 0;

// Motor control
int16_t error = 0, lastError = 0;
uint32_t lastTime = 0;

// Sensor variables
int8_t sensor_logic = 0;
uint16_t sensor1 = 0, sensor2 = 0;

// Navigation modes
#define BLK 0
#define WHT 1
#define RGT 2
#define LFT 3
#define FWD 4
#define BWD 5

// Sensor logic modes
#define NONE 0
#define DIRECT 1
#define EQ 2
#define OR 3
#define XOR 4

// Counter modes
#define TIMER 0
#define ENCODERR 1
#define ENCODERL 2

// Follow modes
#define FLC 0  // Follow Line Center
#define FLR 1  // Follow Line Right
#define FLL 2  // Follow Line Left

#define off 0
#define on 1

int8_t line;
char buff[16];

void setup() {
  Serial.begin(9600);
  
  // Initialize pins
  initButtons();
  initMotors();
  initSensors();
  
  // Initialize display
  Wire.begin();
  lcd.begin();
  lcd.setPowerSave(0);
  lcd.setFlipMode(1);
  lcd.setFont(u8x8_font_chroma48medium8_r);
  
  // Load configuration from EEPROM
  loadConfig();
  
  // Check if factory reset is needed (both OK and BACK pressed)
  if (!digitalRead(BTN_OK) && !digitalRead(BTN_BACK)) {
    factoryReset();
  }
  
  digitalWrite(LED_PIN, HIGH);
  lcd.clear();
}

void loop() {
  if (!digitalRead(BTN_OK)) {
    menu();
    digitalWrite(LED_PIN, HIGH);
  }

  if (!digitalRead(BTN_BACK)) {
    runProgram();
  }
  
  standby();
}

void initButtons() {
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  pinMode(BTN_PLUS, INPUT_PULLUP);
  pinMode(BTN_MINUS, INPUT_PULLUP);
}

void initMotors() {
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  setMotor(0, 0);
}

void initSensors() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  digitalWrite(LED_PIN, LOW);
}

void setMotor(int16_t leftSpeed, int16_t rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_L_DIR, HIGH);
    analogWrite(MOTOR_L_PWM, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_L_DIR, LOW);
    analogWrite(MOTOR_L_PWM, constrain(-leftSpeed, 0, 255));
  }
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_R_DIR, HIGH);
    analogWrite(MOTOR_R_PWM, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_R_DIR, LOW);
    analogWrite(MOTOR_R_PWM, constrain(-rightSpeed, 0, 255));
  }
}

uint16_t readSensor() {
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100);
  
  bitSensor = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    if (sensorValues[i] > config.sensorRef[i]) {
      bitSet(bitSensor, i);
    } else {
      bitClear(bitSensor, i);
    }
  }
  
  digitalWrite(LED_PIN, LOW);
  
  // Invert if following white line
  if (line == WHT) {
    bitSensor = ~bitSensor & 0x1F; // Mask to 5 bits
  }
  
  return bitSensor;
}

// Check for finish line (all sensors detect line)
bool isFinishLine() {
  uint16_t sensors = readSensor();
  return (sensors == 0x1F); // All 5 sensors detect line (11111)
}

// Calculate weighted position for PID control
int16_t getLinePosition() {
  uint16_t sensors = readSensor();
  
  // Weighted position calculation for 5 sensors
  // Position ranges from -2000 (far left) to +2000 (far right)
  int32_t weightedSum = 0;
  int32_t sum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (bitRead(sensors, i)) {
      weightedSum += (i - 2) * 1000; // -2000, -1000, 0, 1000, 2000
      sum++;
    }
  }
  
  if (sum == 0) {
    // No line detected, return last error position
    return (lastError > 0) ? 2000 : -2000;
  }
  
  return weightedSum / sum;
}

// Follow Line Center - PID control for center following
void flc(uint16_t SPEED, int8_t pd) {
  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  
  if (interval >= config.Ts) {
    // Check for finish line
    if (config.stopAtFinish && isFinishLine()) {
      setMotor(0, 0);
      digitalWrite(LED_PIN, LOW);
      lcd.clear();
      lcd.drawString(0, 3, "FINISH LINE!");
      lcd.drawString(0, 4, "STOPPED");
      while (digitalRead(BTN_BACK)) {
        delay(50);
      }
      return;
    }
    
    error = getLinePosition();
    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = (error * config.kp[pd] / 100) + (rateError * config.kd[pd] / 100);
    SPEED = map(SPEED, 0, 100, 0, 255);
    
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(config.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(config.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

// Follow Line Right - Biased toward right edge
void flr(uint16_t SPEED, int8_t pd) {
  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  
  if (interval >= config.Ts) {
    if (config.stopAtFinish && isFinishLine()) {
      setMotor(0, 0);
      return;
    }
    
    int16_t position = getLinePosition();
    error = position + 500; // Bias toward right edge
    
    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = (error * config.kp[pd] / 100) + (rateError * config.kd[pd] / 100);
    SPEED = map(SPEED, 0, 100, 0, 255);
    
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(config.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(config.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

// Follow Line Left - Biased toward left edge
void fll(uint16_t SPEED, int8_t pd) {
  uint32_t NOW = millis();
  uint32_t interval = NOW - lastTime;
  
  if (interval >= config.Ts) {
    if (config.stopAtFinish && isFinishLine()) {
      setMotor(0, 0);
      return;
    }
    
    int16_t position = getLinePosition();
    error = position - 500; // Bias toward left edge
    
    int16_t rateError = error - lastError;
    lastError = error;

    int16_t moveVal = (error * config.kp[pd] / 100) + (rateError * config.kd[pd] / 100);
    SPEED = map(SPEED, 0, 100, 0, 255);
    
    int16_t moveLeft = SPEED - moveVal;
    int16_t moveRight = SPEED + moveVal;

    int16_t VMAX = map(config.Vmax, -100, 100, -255, 255);
    int16_t VMIN = map(config.Vmin, -100, 100, -255, 255);

    moveLeft = constrain(moveLeft, VMIN, VMAX);
    moveRight = constrain(moveRight, VMIN, VMAX);
    
    setMotor(moveLeft, moveRight);
    lastTime = NOW;
  }
}

void calibration() {
  digitalWrite(LED_PIN, HIGH);
  
  uint16_t lRef[NUM_SENSORS], hRef[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    lRef[i] = 1023;
    hRef[i] = 0;
  }
  
  lcd.clear();
  lcd.drawString(0, 2, "Calibrating...");
  lcd.drawString(0, 3, "Move over line");
  lcd.drawString(0, 6, "OK: Save");
  lcd.drawString(0, 7, "BACK: Cancel");
  
  delay(200);
  
  while (digitalRead(BTN_OK) && digitalRead(BTN_BACK)) {
    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds(100);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      uint16_t value = analogRead(sensorPins[i]);
      if (value > hRef[i]) hRef[i] = value;
      if (value < lRef[i]) lRef[i] = value;
    }
    
    digitalWrite(LED_PIN, LOW);
    
    // Adjust sensitivity
    if (!digitalRead(BTN_PLUS)) {
      if (++config.sensitivity > 50) config.sensitivity = 50;
      delay(100);
    }
    if (!digitalRead(BTN_MINUS)) {
      if (--config.sensitivity < -50) config.sensitivity = -50;
      delay(100);
    }
    
    sprintf(buff, "Sens: %d", config.sensitivity);
    lcd.drawString(0, 4, buff);
    
    delay(50);
  }
  
  if (!digitalRead(BTN_OK)) {
    // Save calibration
    for (int i = 0; i < NUM_SENSORS; i++) {
      config.sensorRef[i] = (hRef[i] + lRef[i]) / 2;
      config.sensorRef[i] -= (config.sensorRef[i] * config.sensitivity / 100);
    }
    
    lcd.clear();
    lcd.drawString(0, 3, "SAVED!");
    saveConfig();
  } else {
    lcd.clear();
    lcd.drawString(0, 3, "CANCELLED");
  }
  
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  lcd.clear();
}

void sensor_list(int8_t plan, int8_t x) {
  switch (config.SENSOR[plan][x]) {
    case 0: sensor_logic = NONE; break;
    case 1: sensor_logic = DIRECT; break;
    case 2: sensor1 = 0b00000; sensor_logic = EQ; break; // No sensors
    case 3: sensor1 = 0b11111; sensor_logic = EQ; break; // All sensors (finish line)
    case 4: sensor1 = 0b00001; sensor_logic = OR; break; // Rightmost sensor
    case 5: sensor1 = 0b00100; sensor_logic = OR; break; // Center sensor
    case 6: sensor1 = 0b10000; sensor_logic = OR; break; // Leftmost sensor
    case 7: sensor1 = 0b11111; sensor_logic = OR; break; // Any sensor
    case 8: sensor1 = 0b00011; sensor_logic = OR; break; // Right two sensors
    case 9: sensor1 = 0b11000; sensor_logic = OR; break; // Left two sensors
    case 10: sensor1 = 0b01110; sensor_logic = OR; break; // Center three sensors
    // Add more combinations as needed
  }
}

void runProgram() {
  // Implementation similar to original but with finish line detection
  digitalWrite(LED_PIN, LOW);
  lcd.clear();
  lcd.drawString(0, 3, "Press BACK");
  lcd.drawString(0, 4, "to start");
  
  while (!digitalRead(BTN_BACK)) {
    delay(100);
  }
  
  digitalWrite(LED_PIN, HIGH);
  lastError = 0;
  lastTime = 0;
  line = config.LINE;
  
  int8_t plan = config.PLAN;
  int8_t x = config.CP[config.CPI];
  uint32_t start_time = millis();
  
  // Main execution loop
  while (digitalRead(BTN_BACK)) {
    sensor_list(plan, x);
    
    switch (sensor_logic) {
      case NONE:
        flc(config.V, config.PID);
        break;
      case DIRECT:
        if (config.I == 0) {
          flc(config.V, config.PID);
        }
        executeAction(plan, x);
        break;
      case EQ:
        flc(config.V, config.PID);
        if (readSensor() == sensor1) {
          executeAction(plan, x);
        }
        break;
      case OR:
        flc(config.V, config.PID);
        if ((readSensor() & sensor1) > 0) {
          executeAction(plan, x);
        }
        break;
      case XOR:
        flc(config.V, config.PID);
        if (((readSensor() & sensor1) > 0) && ((readSensor() & sensor2) > 0)) {
          executeAction(plan, x);
        }
        break;
    }
    
    // Check for finish line if enabled
    if (config.stopAtFinish && isFinishLine()) {
      break;
    }
  }
  
  setMotor(0, 0);
  digitalWrite(LED_PIN, LOW);
  
  uint32_t stop_time = millis();
  float lap_time = (stop_time - start_time) / 1000.0;
  
  lcd.clear();
  lcd.drawString(0, 3, "Time: ");
  lcd.setCursor(6, 3);
  lcd.print(lap_time);
  lcd.drawString(12, 3, " sec");
  
  saveConfig();
  delay(3000);
}

void executeAction(int8_t plan, int8_t x) {
  // Implementation remains similar to original
  setMotor(0, 0);
  digitalWrite(LED_PIN, LOW);
  
  int16_t VL = map(config.LSPEED[plan][x], -100, 100, -255, 255);
  int16_t VR = map(config.RSPEED[plan][x], -100, 100, -255, 255);
  setMotor(VL, VR);
  
  switch (config.COUNTERMODE[plan][x]) {
    case TIMER:
      delay(config.COUNTER[plan][x]);
      break;
  }
  
  setMotor(0, 0);
  lastError = 0;
  
  if (config.ACTION[plan][x] < 2) {
    line = config.ACTION[plan][x];
  }
}

void executeFollowAction(int8_t plan, int8_t x, int16_t count, int8_t speed, int8_t mode) {
  digitalWrite(LED_PIN, HIGH);
  
  switch (mode) {
    case TIMER:
      {
        uint32_t count_now = millis();
        while (millis() - count_now < (count * 50)) {
          switch (config.FOLLOWMODE[plan][x]) {
            case FLR: flr(speed, config.pid[plan][x]); break;
            case FLL: fll(speed, config.pid[plan][x]); break;
            default: flc(speed, config.pid[plan][x]); break;
          }
          if (!digitalRead(BTN_BACK)) break;
        }
      }
      break;
  }
}

void factoryReset() {
  // Initialize default configuration
  for (int8_t i = 0; i < MAXPLAN; i++) {
    for (int8_t j = 0; j < 100; j++) {
      config.ACTION[i][j] = 0;
      config.SENSOR[i][j] = 0;
      config.RSPEED[i][j] = 0;
      config.LSPEED[i][j] = 0;
      config.COUNTERMODE[i][j] = TIMER;
      config.COUNTER[i][j] = 0;
      config.VA[i][j] = 50;
      config.COUNTA[i][j] = 0;
      config.COUNTERAMODE[i][j] = TIMER;
      config.VB[i][j] = 50;
      config.COUNTB[i][j] = 0;
      config.COUNTERBMODE[i][j] = TIMER;
      config.pid[i][j] = 2;
      config.FOLLOWMODE[i][j] = FLC;
      config.JUMP[i][j] = off;
      config.jumpP[i][j] = 0;
      config.jumpI[i][j] = 0;
    }
    config.STOP[i] = 99;
  }
  
  for (int8_t i = 0; i < 10; i++) {
    config.CP[i] = 0;
    config.Va[i] = 0;
    config.Ta[i] = 0;
  }
  
  // Default sensor references (mid-range values)
  for (int i = 0; i < NUM_SENSORS; i++) {
    config.sensorRef[i] = 512;
  }
  
  config.kp[0] = 10; config.kp[1] = 13; config.kp[2] = 20;
  config.kd[0] = 20; config.kd[1] = 25; config.kd[2] = 50;
  config.PID = 2;
  config.V = 40;
  config.Vmin = -70; config.Vmax = 100;
  config.Ts = 5;
  config.PLAN = 0;
  config.I = 0;
  config.CPI = 0;
  config.LINE = BLK;
  config.sensitivity = 0;
  config.Dly = 500;
  config.stopAtFinish = true; // Enable finish line detection by default
  
  lcd.drawString(0, 2, " FORMATTING...  ");
  saveConfig();
  lcd.clear();
}

void loadConfig() {
  EEPROM.get(0, config);
}

void saveConfig() {
  EEPROM.put(0, config);
}

void standby() {
  delay(100);
}

void menu() {
  lcd.clear();
  lcd.drawString(0, 0, "MENU");
  lcd.drawString(0, 2, "1. Calibrate");
  lcd.drawString(0, 3, "2. Settings");
  lcd.drawString(0, 4, "3. Run");
  
  while (!digitalRead(BTN_OK)) {
    if (!digitalRead(BTN_UP)) {
      calibration();
      break;
    }
    delay(50);
  }
  delay(200);
}
