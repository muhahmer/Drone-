#include <ESP32Servo.h>
#include <Wire.h>
#include <AlfredoCRSF.h>

#include <MPU6050_light.h>
MPU6050 mpu(Wire);

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
float ground_alt = 0;

#include <Adafruit_VL53L1X.h>
Adafruit_VL53L1X front_laser;
Adafruit_VL53L1X back_laser;

#include <TinyGPSPlus.h>
TinyGPSPlus gps;

AlfredoCRSF crsf;
const byte GPS_TX = 2;
const byte GPS_RX = 5;

#include <SPI.h>
#include <MAX7456.h>

const byte OSD_START   = 27;
const byte OSD_DATA = 23;
const byte OSD_CLOCK = 4;

MAX7456 osd(OSD_START);



// GPS and breadcumb trail 
double trail_lat;
double trail_lon;
float  trail_alt;
int trail_count = 0;

unsigned long last_crumb_time = 0;
unsigned long lost_time = 0;
bool rth_active = false;

// Motors
const byte Motor1 = 12;
const byte Motor2 = 13;
const byte Motor3 = 14;
const byte Motor4 = 15;

Servo M1;
Servo M2;
Servo M3;
Servo M4;

// 2.4GHz Receiver
const byte ELRS_RX = 17;
const byte ELRS_TX = 16;

// Camera tilt servo
const byte Camera_tilt = 33;
Servo camera_servo;

// Sensors
const byte Bat_Sensor = 34; // Battery sensor
const byte I2C_SDA = 21;
const byte I2C_SCL = 22;
const byte IMU_INT = 19;

// Distance Sensor 1 (front)
const byte Front_SDA = 26;
const byte Front_SCL = 32;
int front_distance = 4000;

// Distance Sensor 2 (back)
const byte Back_SDA = 25;
const byte Back_SCL = 18;
int back_distance = 4000;

// Variables to store incoming controller data
int joy_x = 1500;
int joy_y = 1500;
bool takeoff_cmd = false;
int cam_angle = 30;
float Altitude = 0;
int Speed = 0;
float gyro_x = 0;
float gyro_y = 0;
int bat_percent = 100;
float target_alt = 1.0;
int base_throttle = 1000;
float total_trip = 0;
int joy_up_down = 1500;
int joy_yaw = 1500;


void setup() {
  Serial2.begin(420000, SERIAL_8N1, ELRS_RX, ELRS_TX);
  crsf.begin(Serial2);
  
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);


  camera_servo.attach(Camera_tilt);
  camera_servo.write(30);

  M1.attach(Motor1, 1000, 2000);
  M2.attach(Motor2, 1000, 2000);
  M3.attach(Motor3, 1000, 2000);
  M4.attach(Motor4, 1000, 2000);

  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin();
  mpu.calcOffsets();

  bmp.begin(0x76);
ground_alt = bmp.readAltitude(1013.25);

Wire1.begin(Front_SDA, Front_SCL);
front_laser.begin(0x29, &Wire1);
front_laser.startRanging();

Wire1.begin(Back_SDA, Back_SCL);
back_laser.begin(0x29, &Wire1);
back_laser.startRanging();

SPI.begin(OSD_CLOCK, -1, OSD_DATA, OSD_START);
osd.begin();
osd.clear();

}

void loop() {
  crsf.update();
  Read_Reciver();
  Sensors();
  Return_To_Home();
  Drone_Control();
  Camera_Control();
}

void Read_Reciver() {
  if (crsf.isLinkUp()) {
    joy_x       = crsf.getChannel(1); 
    joy_y       = crsf.getChannel(2); 
    joy_up_down = crsf.getChannel(3); 
    joy_yaw     = crsf.getChannel(4); 
    takeoff_cmd = (crsf.getChannel(5) > 1500);
    cam_angle   = crsf.getChannel(6);

    lost_time = millis();
    rth_active = (crsf.getChannel(7) > 1500);
  } else {
    joy_x = 1500;
    joy_y = 1500;
    joy_up_down = 1500;
    joy_yaw = 1500;
    if (takeoff_cmd && millis() - lost_time >= 5000) {
      rth_active = true;
    }
  }
}


void Camera_Control(){
  int angle = map(cam_angle, 1000, 2000, 0, 60);
  angle = constrain(angle, 0, 60);
  camera_servo.write(angle);

// Batery Percentage
  osd.setCursor(1, 1);
  osd.print("BAT:");
  osd.print(bat_percent);
  osd.print("% ");

  // Altitude
  osd.setCursor(18, 1);
  osd.print("ALT:");
  osd.print(Altitude, 1);
  osd.print("M ");

  // Return to home 
  osd.setCursor(10, 7);
  if (rth_active) {
    osd.print("RTH ACTIVE!");
  } else {
    osd.print("           ");
  }

  // Distance Traveleed and distance from takeoff spot 
  int dist_home = 0;
  if (trail_count > 0 && gps.location.isValid()) {
    dist_home = gps.distanceBetween(trail_lat, trail_lon, gps.location.lat(), gps.location.lng());
  }

  osd.setCursor(1, 14);
  osd.print("DIST:");
  osd.print(dist_home);
  osd.print("M TRIP:");
  osd.print((int)total_trip);
  osd.print("M ");



}

void Sensors() {
  mpu.update();
  gyro_x = mpu.getAngleX();
  gyro_y = mpu.getAngleY();

  Altitude = bmp.readAltitude(1013.25) - ground_alt;

  float raw_bat = analogRead(Bat_Sensor) * (3.3 / 4095.0) * 5.7;
  bat_percent = constrain(map(raw_bat * 100, 1320, 1680, 0, 100), 0, 100);
  
  Wire1.begin(Front_SDA, Front_SCL);
if (front_laser.dataReady()) {
  front_distance = front_laser.distance();
  front_laser.clearInterrupt();
}

Wire1.begin(Back_SDA, Back_SCL);
if (back_laser.dataReady()) {
  back_distance = back_laser.distance();
  back_laser.clearInterrupt();
}
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid() && gps.satellites.value() >= 3) {
    Speed = gps.speed.mps();

    if (takeoff_cmd && !rth_active && millis() - last_crumb_time >= 1000) {
    if (trail_count < 1000) {
     trail_lat[trail_count] = gps.location.lat();
     trail_lon[trail_count] = gps.location.lng();
     trail_alt[trail_count] = Altitude;
     if (trail_count > 0) {
       total_trip += gps.distanceBetween(trail_lat[trail_count - 1], trail_lon[trail_count - 1], gps.location.lat(), gps.location.lng());
     }
     trail_count++;
   }

  }

}
}

void Return_To_Home() {
  if (!rth_active) return; 


  if (trail_count <= 0) {
    takeoff_cmd = false;
    rth_active = false;
    return;
  }


  int target_idx = trail_count - 1;
  double target_lat = trail_lat[target_idx];
  double target_lon = trail_lon[target_idx];
  target_alt = trail_alt[target_idx];


  double dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), target_lat, target_lon);
  double course = gps.courseTo(gps.location.lat(), gps.location.lng(), target_lat, target_lon);


  if (dist < 2.0) {
    trail_count--;
    return;
  }


  float rad = radians(course);
  joy_y = 1500 + (cos(rad) * 150);
  joy_x = 1500 + (sin(rad) * 150);
}

void Drone_Control() {
  if (takeoff_cmd) {
    if (!rth_active) {
      if (joy_up_down > 1550) target_alt += 0.02;
      else if (joy_up_down < 1450) target_alt -= 0.02;
      target_alt = constrain(target_alt, 0.3, 50.0);
    }

    if (Altitude < target_alt - 0.1) base_throttle += 2;
    else if (Altitude > target_alt + 0.1) base_throttle -= 2;
    base_throttle = constrain(base_throttle, 1150, 1900);
  } else {
    target_alt = 1.0;
    if (Altitude > 0.15 && base_throttle > 1100) {
      base_throttle -= 2;
    } else {
      base_throttle = 1000;
      M1.writeMicroseconds(1000);
      M2.writeMicroseconds(1000);
      M3.writeMicroseconds(1000);
      M4.writeMicroseconds(1000);
      return;
    }
  }

  float target_roll  = map(joy_x, 1000, 2000, -35, 35);
  float target_pitch = map(joy_y, 1000, 2000, -35, 35);
  int yaw_cmd        = map(joy_yaw, 1000, 2000, -150, 150);

  if (front_distance < 600 && target_pitch > 0) target_pitch = -15;
  if (back_distance < 600 && target_pitch < 0)  target_pitch = 15;

  float kp = 3.0;
  int roll_corr  = (target_roll - gyro_x) * kp;
  int pitch_corr = (target_pitch - gyro_y) * kp;


  int m1_speed = constrain(base_throttle - roll_corr + pitch_corr - yaw_cmd, 1050, 2000);
  int m2_speed = constrain(base_throttle - roll_corr - pitch_corr + yaw_cmd, 1050, 2000);
  int m3_speed = constrain(base_throttle + roll_corr + pitch_corr + yaw_cmd, 1050, 2000);
  int m4_speed = constrain(base_throttle + roll_corr - pitch_corr - yaw_cmd, 1050, 2000);

  M1.writeMicroseconds(m1_speed);
  M2.writeMicroseconds(m2_speed);
  M3.writeMicroseconds(m3_speed);
  M4.writeMicroseconds(m4_speed);
}
