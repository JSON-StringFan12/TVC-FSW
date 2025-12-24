#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>

// PID Gains (Applies to pitch and yaw)
float Kp = 0.65;
float Ki = 0.0;
float Kd = 0.05;

// README: 
// this is for primary teensey
// double check pitch yaw reference, could be 0 or 90 depending on how you define it
// for auxillary teensey, the SD card pin is 15, and make sure read / write format is FAT32

const int BuzzerPin = 15; // Servo pins are specified in servo parameters in setup
const float EndTime = 40; // TVC active window (sec after launch)

struct State {
    float yaw;       // Z axis (degrees)
    float pitch;     // Y axis (degrees)
    float roll;      // X axis (degrees)
    float altitude;  // Altitude (m)
    float accel_z;   // Z axis acceleration (m/s^2)
    float accel_y;   // Z axis acceleration (m/s^2)
    float accel_x;   // X axis acceleration (m/s^2)
    float TVCPitch;
    float TVCYaw;
    float time;      // seconds since LaunchTime
    float launchFlag; // 0.0 = not launched, 1.0 = launched
};

struct ServoParams {
  int pin;
  float target;
  float integral;
  float prev_error;
  float signScalar;
  float a;   // Four bar linkage a length (mm)
  float b;   // Four bar linkage b length (mm)
  float c;   // Four bar linkage c length (mm)
  float d;   // Four bar linkage d length (mm)
  float phi; // Angle of fixed linkage (rad)
};

ServoParams PitchParams;
ServoParams YawParams;

bool LaunchDetected = false;
unsigned long LaunchTime = 0;

State GlobalState;
State LastState;

Servo PitchServo;
Servo YawServo;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP085 bmp;

// ----------------- BUZZER / STATUS -----------------
void ActivateBuzzer(int BeepTime, int WaitTime, int Tone) {
  tone(BuzzerPin, Tone);
  delay(BeepTime);
  noTone(BuzzerPin);
  delay(WaitTime);
}

// 3 short beeps = All OK (input=0)
// 4 long beeps = IMU error (input=1)
// 3 long beeps = SD write error (input=2)
// 2 long beeps = Altimeter error (input=3)
void ReportStatus(int status) {
  switch (status) {
    case 0:
      for (int i = 0; i < 3; i++) {
        ActivateBuzzer(200, 100, 1500);
      }
      break;
    case 1:
      for (int i = 0; i < 4; i++) {
        ActivateBuzzer(600, 100, 500);
      }
      break;
    case 2:
      for (int i = 0; i < 3; i++) {
        ActivateBuzzer(600, 100, 500);
      }
      break;
    case 3:
      for (int i = 0; i < 2; i++) {
        ActivateBuzzer(600, 100, 500);
      }
      break;
  }
}

// ----------------- SENSORS -----------------
void InitIMU() {
  Wire.begin();
  if (!bno.begin()) {
    // Sensor not detected
    ReportStatus(1);
    while (1) {
      delay(1000);
    }
  }
  // optional: use external crystal for better precision
  bno.setExtCrystalUse(true);
  delay(1000);
}

void InitAltimeter() {
  if (!bmp.begin()) {
    // Sensor not detected
    ReportStatus(3);
    while (1) {
      delay(1000);
    }
  }
}

// Gets the state of the vehicle from IMU + altimeter data
static State GetState() {
  State NewState;

  // Orientation (quaternion)
  imu::Quaternion q = bno.getQuat();
  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  // Roll (X-axis rotation)
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  NewState.roll = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (Y-axis rotation)
  float fx = 2.0f * (w * x + y * z);
  float fy = 2.0f * (w * y - x * z);
  float fz = 1.0f - 2.0f * (x * x + y * y);
  float angle = acosf(fz);
  NewState.pitch = angle;

  // Yaw (Z-axis rotation)
  NewState.yaw = atan2f(fy, fx);

  // radians → degrees
  NewState.roll  *= 180.0f / M_PI;
  NewState.pitch *= 180.0f / M_PI; // README: double check reference and fc orientation
  NewState.yaw   *= 180.0f / M_PI;

  // Linear acceleration (ZYX)
  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  NewState.accel_z = la.z();
  NewState.accel_y = la.y();
  NewState.accel_x = la.x();

  // Altitude
  NewState.altitude = bmp.readAltitude();

  // Time since launch (sec)
  NewState.time = (millis() - LaunchTime) / 1000.0f;

  // launchFlag will be filled in by loop()
  NewState.launchFlag = 0.0f;

  return NewState;
}

// ----------------- CONTROL / LINKAGE -----------------
float ComputePID(float error, float &integral, float &prev_error,
                 float Kp, float Ki, float Kd, float dt) {
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  prev_error = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// Four-bar linkage solver
float SolveFourBar(double a, double b, double c, double d,
                   double theta2, double phi) {
  // Ground pivot location
  double Dx = d * cos(phi);
  double Dy = d * sin(phi);

  // Driver endpoint A'
  double Ax = a * cos(theta2);
  double Ay = a * sin(theta2);

  // Distance between A' and D
  double R = sqrt((Ax - Dx)*(Ax - Dx) + (Dy - Ay)*(Dy - Ay));

  // Law of cosines for angle at link c
  double cos_gamma = (b*b + c*c - R*R) / (2*b*c);
  if (cos_gamma < -1.0 || cos_gamma > 1.0) return 0.0;

  double gamma = acos(cos_gamma);
  double baseAngle = atan2(Dy - Ay, Dx - Ax);

  // Law of sines for second triangle angle (thetaB)
  double thetaB = asin(c / R * sin(gamma));

  double theta4a = (3.141592 - gamma - thetaB) - baseAngle;
  return theta4a;
}

float GetCMD(ServoParams P, float err, float dt, float &Log) {
  float cmd = ComputePID(err, P.integral, P.prev_error, Kp, Ki, Kd, dt);

  // Constrain and apply sign
  cmd = constrain(cmd, -7.5, 7.5);
  cmd *= P.signScalar;
  Log = cmd;

  // Find true neutral point angle of servo
  float neutral_point = SolveFourBar(P.a, P.b, P.c, P.d, M_PI / 2.0, P.phi);

  // Find change from neutral point
  float delta = SolveFourBar(P.a, P.b, P.c, P.d,
                             M_PI / 2.0 + (cmd * M_PI / 180.0), P.phi)
                - neutral_point;

  // Convert to servo command in degrees
  cmd = 90.0 + delta * 180.0 / M_PI;
  return cmd;
}

// ----------------- TELEMETRY (to logger Teensy) -----------------
void SendPacket(const State &s) {
  const uint8_t header[2] = {0xAA, 0x55};
  Serial1.write(header, 2);
  Serial1.write(reinterpret_cast<const uint8_t*>(&s), sizeof(State));
}

// print state (text)
void PrintStateDebug(const State &s) {
  Serial.print("t"); Serial.print(s.time, 3);
  Serial.print("  launch:"); Serial.print(s.launchFlag, 0);
  Serial.print("  yaw:"); Serial.print(s.yaw, 2);
  Serial.print("  pitch:"); Serial.print(s.pitch, 2);
  Serial.print("  roll:"); Serial.print(s.roll, 2);
  Serial.print("  alt:"); Serial.print(s.altitude, 2);
  Serial.print("  ax:"); Serial.print(s.accel_x, 2);
  Serial.print("  ay:"); Serial.print(s.accel_y, 2);
  Serial.print("  az:"); Serial.print(s.accel_z, 2);
  Serial.print("  TVC_p:"); Serial.print(s.TVCPitch, 2);
  Serial.print("  TVC_y:"); Serial.println(s.TVCYaw, 2);
}

// CSV line for Serial Plotter
void PrintStatePlot(const State &s) {
  Serial.print(s.time, 3);      Serial.print(',');
  Serial.print(s.launchFlag,0); Serial.print(',');
  Serial.print(s.yaw, 3);       Serial.print(',');
  Serial.print(s.pitch, 3);     Serial.print(',');
  Serial.print(s.roll, 3);      Serial.print(',');
  Serial.print(s.altitude, 3);  Serial.print(',');
  Serial.print(s.accel_x, 3);   Serial.print(',');
  Serial.print(s.accel_y, 3);   Serial.print(',');
  Serial.print(s.accel_z, 3);   Serial.print(',');
  Serial.print(s.TVCPitch, 3);  Serial.print(',');
  Serial.println(s.TVCYaw, 3);  // newline ends the sample
}

// ----------------- SETUP / LOOP -----------------
void setup() {
  Serial.begin(115200);    // USB serial to PC
  Serial1.begin(115200);   // telemetry to logger Teensy

  pinMode(BuzzerPin, OUTPUT);

  // Servo param setup
  YawParams.pin = 5;
  YawParams.target = 90;
  YawParams.integral = 0;
  YawParams.prev_error = 0;
  YawParams.signScalar = -1;
  YawParams.a = 35.0;
  YawParams.b = 24.75;
  YawParams.c = 10.45;
  YawParams.d = 39.39;
  YawParams.phi = 0.7833;

  PitchParams.pin = 4;
  PitchParams.target = 90;
  PitchParams.integral = 0;
  PitchParams.prev_error = 0;
  PitchParams.signScalar = -1;
  PitchParams.a = 31.5;
  PitchParams.b = 25.75;
  PitchParams.c = 10.45;
  PitchParams.d = 39.14;
  PitchParams.phi = 2.38;

  PitchServo.attach(PitchParams.pin);
  YawServo.attach(YawParams.pin);

  PitchServo.write(90);
  YawServo.write(90);

  InitIMU();
  InitAltimeter();
  ReportStatus(0);

  // little servo wiggle
  PitchServo.write(80);  delay(250);
  PitchServo.write(100); delay(250);
  PitchServo.write(90);  delay(250);
  YawServo.write(80);    delay(250);
  YawServo.write(100);   delay(250);
  YawServo.write(90);
}

void loop() {
  delay(10);

  if (!LaunchDetected) {
    // Pre-launch: watch accel_x, but STILL send telemetry
    LastState = GlobalState;
    GlobalState = GetState();
    GlobalState.launchFlag = LaunchDetected ? 1.0f : 0.0f;

    if (GlobalState.accel_x >= 2.5) {
      LaunchDetected = true;
      LaunchTime = millis();
      Serial.println("Launch Detected");
    }

    // Send pre-launch telemetry + optional debug
    SendPacket(GlobalState);
    PrintStatePlot(GlobalState);

  } else {
    // Launch detected
    LastState = GlobalState;
    GlobalState = GetState();
    GlobalState.launchFlag = LaunchDetected ? 1.0f : 0.0f;

    if (GlobalState.time <= EndTime) {
      float pitch_error = PitchParams.target - GlobalState.pitch;
      float yaw_error   = PitchParams.target - GlobalState.yaw;

      float dt = GlobalState.time - LastState.time;
      if (dt <= 0) dt = 0.01f; // safety

      // Move the servos according to the angle command returned by GetCMD()
      PitchServo.write(GetCMD(PitchParams, pitch_error, dt, GlobalState.TVCPitch));
      YawServo.write(GetCMD(YawParams, yaw_error, dt, GlobalState.TVCYaw));
    } else {
      // End of TVC window; one long 2 s beep when going back to idle
      LaunchDetected = false;
    }

    // ALWAYS send telemetry, even after EndTime
    SendPacket(GlobalState);
    PrintStateDebug(GlobalState);
    // or: PrintStatePlot(GlobalState);
  }
}
