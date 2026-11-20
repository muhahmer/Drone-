#include <ESP32Servo.h>
#include <Wire.h>
#include <AlfredoCRSF.h>
#include <ESP32Encoder.h>
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
#include <MPU6050_light.h>
MPU6050 mpu(Wire);



AlfredoCRSF crsf;
// Joystick
const byte X_axis = 32;
const byte Y_axis = 33;
const byte Joy_button = 25;

// Rotary Encoder 1 (screen tilt 1)
const byte rotary_count_1 = 26;
const byte rotary_direction_1 = 27;
const byte rotary_button_1 = 12;

// Rotary Encoder 2 (screen tilt 2)
const byte rotary_count_2 = 19;
const byte rotary_direction_2 = 23;
const byte rotary_button_2 = 14;

// Rotary Encoder 3 (Camera tilt rotary 3)
const byte rotary_count_3 = 16;
const byte rotary_direction_3 = 17;
const byte rotary_button_3 = 4;

// 2.4GHz Transmitter
const byte RX = 2;
const byte TX = 5;

// Servo Left and Right
const byte servo_left_right = 15;
Servo servo_lef_right;

// Servo forwards backwards
const byte servo_forward_backward = 13;
Servo servo_for_back;

// SDA PIN for tilt sensor
const byte I2C_SDA = 21;

// SCL PIN for tilt sensor
const byte I2C_SCL = 22;

bool is_flying = false;
bool last_joy_btn = HIGH;
bool manual_rth = false;
bool last_rth_btn = HIGH;


int X_value;
int Y_value;
int base_angle_1 = 90;
int base_angle_2 = 90;
crsf_channels_t channels = {0};

void setup() {

Wire.begin(I2C_SDA, I2C_SCL);
Serial.begin(115200);

// Joystick 
pinMode(X_axis, INPUT);
pinMode(Y_axis, INPUT);
pinMode(Joy_button, INPUT_PULLUP);

// Rotary Encoder 1
pinMode(rotary_count_1, INPUT_PULLUP);
pinMode(rotary_direction_1, INPUT_PULLUP);
pinMode(rotary_button_1, INPUT_PULLUP);

// Rotary Encoder 2
pinMode(rotary_count_2, INPUT_PULLUP);
pinMode(rotary_direction_2, INPUT_PULLUP);
pinMode(rotary_button_2, INPUT_PULLUP);

// Rotary Encoder 3
pinMode(rotary_count_3, INPUT_PULLUP);
pinMode(rotary_direction_3, INPUT_PULLUP);
pinMode(rotary_button_3, INPUT_PULLUP);

// 2.4GHz Transmitter
Serial2.begin(400000, SERIAL_8N1, RX, TX);
crsf.begin(Serial2);

// Servos
servo_lef_right.attach(servo_left_right);
servo_for_back.attach(servo_forward_backward);
servo_lef_right.write(90);
servo_for_back.write(90);

ESP32Encoder::useInternalWeakPullResistors = UP;
encoder1.attachHalfQuad(rotary_count_1, rotary_direction_1);
encoder2.attachHalfQuad(rotary_count_2, rotary_direction_2);
encoder3.attachHalfQuad(rotary_count_3, rotary_direction_3);

mpu.begin();
mpu.calcOffsets();


}

void loop() {
  crsf.update();
  Joystick();
  Rotary_Encoders();
  Screen_tilt();
  Camera_tilt();
  crsf.queuePacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
  delay(10);
}


void Joystick() {
  X_value = analogRead(X_axis);
  Y_value = analogRead(Y_axis);

  X_value = map(X_value, 0, 4095, 172, 1811);
  Y_value = map(Y_value, 0, 4095, 172, 1811);

  bool current_joy_btn = digitalRead(Joy_button);
  if (current_joy_btn == LOW && last_joy_btn == HIGH) {
    is_flying = !is_flying;
    if (!is_flying) manual_rth = false;
    delay(50);
  }
  last_joy_btn = current_joy_btn;

  channels.ch2 = Y_value; 
  channels.ch3 = X_value; 
  channels.ch4 = is_flying ? 1811 : 172; 
}



void Rotary_Encoders(){
  if (digitalRead(rotary_button_1) == LOW) encoder1.clearCount();
  if (digitalRead(rotary_button_2) == LOW) encoder2.clearCount();

  base_angle_1 = constrain(90 + encoder1.getCount(), 0, 180);
  base_angle_2 = constrain(90 + encoder2.getCount(), 0, 180);
}

void Screen_tilt() {
  mpu.update();

  int tilt_x = mpu.getAngleX();
  int tilt_y = mpu.getAngleY();

  int final_angle_1 = constrain(base_angle_1 - tilt_x, 0, 180);
  int final_angle_2 = constrain(base_angle_2 - tilt_y, 0, 180);
  servo_lef_right.write(final_angle_1);
  servo_for_back.write(final_angle_2);

  int send_roll  = constrain(map(tilt_x, -35, 35, 172, 1811), 172, 1811);
  int send_pitch = constrain(map(tilt_y, -35, 35, 172, 1811), 172, 1811);
  channels.ch0 = send_roll;
  channels.ch1 = send_pitch;
}

void Camera_tilt() {
  int camera_angle = constrain(30 + encoder3.getCount(), 0, 60);
  channels.ch5 = map(camera_angle, 0, 60, 172, 1811); // Camera Tilt (Channel 6)

  bool current_rth_btn = digitalRead(rotary_button_3);
  if (current_rth_btn == LOW && last_rth_btn == HIGH) {
    manual_rth = !manual_rth;
    delay(50);
  }
  last_rth_btn = current_rth_btn;
  channels.ch6 = manual_rth ? 1811 : 172;
}
