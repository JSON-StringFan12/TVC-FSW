#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP085.h>

#include <Servo.h>

const int BuzzerPin = 15; // Servo pins are specified in servo parameters in setup

const float EndTime = 7.5;

struct State {
    float yaw;    // Z axis (degrees)
    float pitch;  // Y axis (degrees)
    float roll;   // X axis (degrees)
    float altitude; // Altitude (m)
    float accel_z; // Z axis acceleration (m/s^2)
    float accel_y; // Y axis acceleration (m/s^2)
    float accel_x; // X axis acceleration (m/s^2)
    float TVCPitch;
    float TVCYaw;
    float time;
};

struct ServoParams {
  int pin;
  float target;
  float integral;
  float prev_error;
  float signScalar;
  float a; // Four bar linkage a length (mm)
  float b; // Four bar linkage b length (mm)
  float c; // Four bar linkage c length (mm)
  float d; // Four bar linkage d length (mm)
  float phi; // Angle of fixed linkage (rad)
};

ServoParams PitchParams;

ServoParams YawParams;

bool LaunchDetected = false;
unsigned long LaunchTime = 0;

// PID Gains (Applies to pitch and yaw)
float Kp = 0.7;
float Ki = 0.0;
float Kd = 0.25;

State GlobalState;
State LastState;

Servo PitchServo;
Servo YawServo;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP085 bmp;

// Activate the buzzer depending on the int parsed to it. 
// 3 short beeps = All OK (input=0)
// 4 long beeps = IMU error (input=1)
// 3 long beeps = SD write error (input=2)
// 2 long beeps = Altimeter error (input=3)
void ReportStatus(int status) {
  switch (status) {
    case 0:
      for (int i=0; i<3; i++) {
        ActivateBuzzer(200, 100, 1500);
      }
      break;
    case 1:
      for (int i=0; i<4; i++) {
        ActivateBuzzer(600, 100, 500);
      }
      break;
    case 2:
      for (int i=0; i<3; i++) {
        ActivateBuzzer(600, 100, 500);
      }
      break;
  }
}

// Activate the buzzer (at tone Tone Hz) for BeepTime milliseconds and delay WaitTime milliseconds afterwards
void ActivateBuzzer(int BeepTime, int WaitTime, int Tone) {
  tone(BuzzerPin, Tone);
  delay(BeepTime);
  noTone(BuzzerPin);
  delay(WaitTime);
}

// Initialize IMU
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

// Initialize Altimeter
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
// IMU data is converted from quaternions (pitch and yaw are the important ones)
static State GetState() {
  State NewState;

  // Get Orientation
  imu::Quaternion q = bno.getQuat();

  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  // --- Roll (X-axis rotation) ---
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  NewState.roll = atan2f(sinr_cosp, cosr_cosp);

  // --- Pitch (Y-axis rotation) ---
  float fx = 2.0f * (w*x + y*z);
  float fy = 2.0f * (w*y - x*z);
  float fz = 1.0f - 2.0f * (x*x + y*y);

  float angle = acosf(fz);         // radians
  NewState.pitch = angle; // 0–180 degrees

  // --- Yaw (Z-axis rotation) ---
  NewState.yaw = atan2f(fy, fx);   // radians

  // convert to degrees
  NewState.roll  *= 180.0f / M_PI;
  NewState.pitch *= 180.0f / M_PI;
  NewState.yaw   *= 180.0f / M_PI;

  // Get Acceleration (ZYX)
  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
  NewState.accel_z = la.z();
  NewState.accel_y = la.y();
  NewState.accel_x = la.x();

  // Get Altitude
  NewState.altitude = bmp.readAltitude();

  // Get Time Since Launch (time before launch will be very large)
  NewState.time = (millis() - LaunchTime) / 1000.;

  return NewState;
}

// Computes PID for given values
float ComputePID(float error, float &integral, float &prev_error, float Kp, float Ki, float Kd, float dt) {
    // Integral term
    integral += error * dt;

    // Derivative term
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // PID Output
    float output = Kp*error + Ki*integral + Kd*derivative;

    return output;
}

// Solves the four bar linkage for the given TVC
// Inputs lengths and angles of bar (theta2 is desired TVC angle) and outputs the servo angle (w.r.t. horizontal)
// Output needs to be converted to the true servo command angle
float SolveFourBar(double a, double b, double c, double d, double theta2, double phi) {
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
    double thetaB = asin(c/R*sin(gamma));

    double theta4a = (3.141592 - gamma - thetaB) - baseAngle;
    return theta4a;
}

float GetCMD(ServoParams P, float err, float dt, float &Log) {
  // Compute PID command
  float cmd = ComputePID(err, P.integral, P.prev_error, Kp, Ki, Kd, dt);

  // Constrain command and flip its sign (if specified)
  cmd = constrain(cmd, -7.5, 7.5);
  cmd *= P.signScalar;
  Log = cmd;


  // Find true neutral point angle of servo
  float neutral_point = SolveFourBar(P.a, P.b, P.c, P.d, M_PI/2., P.phi);

  // Find change from neutral point
  float delta = SolveFourBar(P.a, P.b, P.c, P.d, M_PI/2. + (cmd * M_PI/180.), P.phi) - neutral_point;
  
  // Add delta to command neutral point
  cmd = 90. + delta * 180.0 / M_PI;

  return cmd;
}

// Send telemetry data to aux. flight computer for SD logging
void SendPacket(const State &p) {
    // Send a header (optional but strongly recommended)
    const uint8_t header[2] = {0xAA, 0x55};
    Serial1.write(header, 2);

    // Send raw struct bytes
    Serial1.write((uint8_t*)&p, sizeof(SensorPacket));
}

void setup() {
  // put your setup code here, to run once:
  YawParams.pin = 5;
  YawParams.target = 90;
  YawParams.integral = 0;
  YawParams.prev_error = 0;
  YawParams.signScalar = 1;
  YawParams.a = 35.0;
  YawParams.b = 24.75;
  YawParams.c = 10.45;
  YawParams.d = 39.39;
  YawParams.phi = .7833;

  PitchParams.pin = 4;
  PitchParams.target = 90;
  PitchParams.integral = 0;
  PitchParams.prev_error = 0;
  PitchParams.signScalar = 1;
  PitchParams.a = 31.5;
  PitchParams.b = 25.75;
  PitchParams.c = 10.45;
  PitchParams.d = 39.14;
  PitchParams.phi = 2.38;

  Serial.begin(9600);
  Serial1.begin(115200);

  pinMode(BuzzerPin, OUTPUT);

  PitchServo.attach(PitchParams.pin);
  YawServo.attach(YawParams.pin);

  PitchServo.write(90);
  YawServo.write(90);

  InitIMU();
  InitAltimeter();
  ReportStatus(0);
  PitchServo.write(80);
  delay(250);
  PitchServo.write(100);
  delay(250);
  PitchServo.write(90);
  delay(250);
  YawServo.write(80);
  delay(250);
  YawServo.write(100);
  delay(250);
  YawServo.write(90);
}

void loop() {
  delay(10);
  // put your main code here, to run repeatedly:
  // See if we have launched
  if (!LaunchDetected) {
    // No launch detected, check if acceleration is high
    LastState = GlobalState;
    GlobalState = GetState();
    if (GlobalState.accel_x >= 2.5) {
      LaunchDetected = true;
      LaunchTime = millis();
      Serial.println("Launch Detected");
    }
  } else {
    // Launch detected
    LastState = GlobalState;
    GlobalState = GetState();
    // Check if past end time (TVC will not work if motor is burned out and we also don't need to log the entire flight's data)
    if (GlobalState.time <= EndTime) {
      float pitch_error = PitchParams.target - GlobalState.pitch;
      float yaw_error   = YawParams.target - GlobalState.yaw;

      float dt = GlobalState.time - LastState.time;

      Serial.println(GlobalState.TVCPitch);

      // Move the servos according to the angle command returned by GetCMD()
      PitchServo.write(GetCMD(PitchParams, pitch_error, dt, GlobalState.TVCPitch));
      YawServo.write(GetCMD(YawParams, yaw_error, dt, GlobalState.TVCYaw));

      SendPacket(GlobalState); // Send state packet to aux. flight computer for SD logging
    } else {
      // Change back to LaunchDetected = false just in case there was a false trigger of the launch detection
      LaunchDetected = false;
    }
  }
}

