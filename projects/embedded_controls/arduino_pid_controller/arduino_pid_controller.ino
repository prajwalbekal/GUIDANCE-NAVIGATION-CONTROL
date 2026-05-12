const int SENSOR_PIN = A0;
const int ACTUATOR_PIN = 9;

float setpoint = 70.0;
float kp = 3.0;
float ki = 0.45;
float kd = 0.18;

float integral = 0.0;
float previousError = 0.0;
unsigned long previousTimeMs = 0;

const float outputMin = 0.0;
const float outputMax = 255.0;
const float integralMin = -120.0;
const float integralMax = 120.0;

float sensorToEngineeringUnits(int adcValue) {
  return (adcValue / 1023.0) * 100.0;
}

float clampValue(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void handleSerialCommand() {
  if (!Serial.available()) return;

  char command = Serial.read();
  float value = Serial.parseFloat();

  if (command == 'S' || command == 's') setpoint = value;
  if (command == 'P' || command == 'p') kp = value;
  if (command == 'I' || command == 'i') ki = value;
  if (command == 'D' || command == 'd') kd = value;

  while (Serial.available()) Serial.read();
}

void setup() {
  pinMode(ACTUATOR_PIN, OUTPUT);
  Serial.begin(115200);
  previousTimeMs = millis();
  Serial.println("time_ms,setpoint,measurement,control,error");
}

void loop() {
  handleSerialCommand();

  unsigned long nowMs = millis();
  float dt = (nowMs - previousTimeMs) / 1000.0;
  if (dt < 0.02) return;
  previousTimeMs = nowMs;

  int raw = analogRead(SENSOR_PIN);
  float measurement = sensorToEngineeringUnits(raw);
  float error = setpoint - measurement;

  integral += error * dt;
  integral = clampValue(integral, integralMin, integralMax);

  float derivative = (error - previousError) / dt;
  previousError = error;

  float control = kp * error + ki * integral + kd * derivative;
  control = clampValue(control, outputMin, outputMax);

  analogWrite(ACTUATOR_PIN, (int)control);

  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(setpoint, 3);
  Serial.print(",");
  Serial.print(measurement, 3);
  Serial.print(",");
  Serial.print(control, 3);
  Serial.print(",");
  Serial.println(error, 3);
}

