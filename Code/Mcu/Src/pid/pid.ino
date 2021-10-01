#include "dc_motor.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"
#include "BluetoothSerial.h"
#include <string>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//#define CUSTOM_SETTINGS
//#define INCLUDE_GAMEPAD_MODULE
//#include <DabbleESP32.h>
#include <Ps3Controller.h>

#define Kp  2000    //3000.0  //3000.0  //2700.0  // 2000 //Ku= 8000
#define Kd  20.0    //10.0    //10.0    //9.0     // 20.0  
#define Ki  5000   //70000.0 //60000.0 //53000.0 //22000    
#define Kyaw 0.03
#define sampleTime  0.01  // 100 Hz
#define zero_targetAngle -0.0  // Calibrated point

uint8_t i;

float targetAngle = 0.0;
float currentAngle = 0.0, prevAngle = 0.0, error = 0.0, prevError = 0.0, errorSum = 0.0, yaw = 0.0, targetYaw = 0.0;
int pwm = 0;
volatile bool controlFlag = false;

bool ps3_joy = false;
bool mit_joy = true;
#define yaw_ascii 89
#define pitch_ascii 80
uint8_t app_data_buff[20];
String app_data;
String app_str_yaw;
float app_float_yaw;
float app_float_pitch;
float fwd = 0;
float rot = 0;

unsigned long pid_count = 0;

int time_ctr = 0;
float ax, az, gy , gz;
float accelPitch, pitch;
float tau = 0.98;

bool stop_command = true;
bool x_down;

BluetoothSerial SerialBT;

void setup() {
  // UART PC connection
  Serial.begin(115200);

  // Motor initialization
  motor_init();

  // IMU init
  imu_setup();

  // Bluetooth init
  //  Dabble.begin("MyEsp32");       //set bluetooth name of your device
  if (ps3_joy){
    ps3_setup();
  }
  
  //Bluetooth Setup
  if (mit_joy){
    SerialBT.begin("Balancio"); //Bluetooth device name
    app_data.reserve(20);
  }
  
  // Timer init. (ISR at 100 Hz)
  timer_init();
}

// ISR at 100 Hz
void IRAM_ATTR onTime() {
  controlFlag = true;
}

void loop() {

  if (controlFlag) {

    if (ps3_joy){
      x_down = Ps3.event.button_down.cross;
      if (x_down) {
        stop_command = abs(stop_command - 1);
      }
    }
    else {stop_command = false;}
    
    if (!stop_command) {
      getAccelGyro(&ax, &az, &gy, &gz);
      accelPitch = atan2(ax, az) * RAD_TO_DEG;
      pitch = (tau) * (pitch + (-gy) * sampleTime) + (1 - tau) * (accelPitch);
      currentAngle = pitch * DEG_TO_RAD;

      error = currentAngle - targetAngle;
      errorSum += error;
      errorSum = constrain(errorSum, -5, 5);
      pwm = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * gy * sampleTime * DEG_TO_RAD;

      if (pwm > 0) //to take into account the static friction of the motors
        pwm += 12;
      if (pwm < 0)
        pwm -= 12;

      pwm = constrain(pwm, -255, 255);

      prevAngle = currentAngle;

      yaw = yaw + gz * DEG_TO_RAD;
      rot = Kyaw * (yaw - targetYaw);
      
      L_motor(pwm + int(rot * 60));
      R_motor(pwm - int(rot * 60));

      if (false) {
        Serial.print("fwd: ");
        Serial.print(fwd);
        Serial.print(" rot: ");
        Serial.print(rot);
        Serial.print("  Pitch:  ");
        Serial.print(currentAngle, 4);
        Serial.print("  Yaw:  ");
        Serial.print(yaw, 4);
        Serial.print("  Gy:  ");
        Serial.print(gy, 4);
        Serial.print("  PWM:  ");
        Serial.print(pwm);
        Serial.print("  Loop time: ");
        time_ctr = micros() - time_ctr;
        Serial.println(time_ctr);
        time_ctr = micros();
      }

    }
    else {
      stop_motor();
    }

    controlFlag = false;

  }
  else {
    return;
  }

  // Get joystick commands
  //  Dabble.processInput();
  //  fwd = GamePad.getYaxisData();
  //  fwd = fwd/7.0;
  //  targetYaw = GamePad.getXaxisData();
  //  targetYaw = targetYaw/7.0;

  // PS3 Joy
  if (ps3_joy) {
      fwd = -Ps3.data.analog.stick.ry / 128.0;
      //rot = Ps3.data.analog.stick.lx / 128.0;
      targetYaw += Ps3.data.analog.stick.lx / 50.0;
  }

  // MIT Inventor
//  if (mit_joy){
//    if (SerialBT.available()) {
//      //app_data[0]=SerialBT.read();
//      app_data=SerialBT.read();
//      if (app_data == yaw_ascii){
//        //app_str_yaw="";
//        Serial.print("Received Yaw: ");
//        for (i=2; i<20; i++){
//          if (app_data[i] == 0x0A){break;}
//          //app_str_yaw += String(app_data[i], DEC);
//          app_str_yaw[i-2] = app_data[i];
//          //app_str_yaw += String(0x0A, DEC);
//          Serial.write(app_data[i]);
//        }     
//        app_float_yaw = app_str_yaw.toFloat();
//        Serial.print(app_float_yaw);
//        //Serial.print(app_str_yaw);
//      }
//      else if (app_data == pitch_ascii){
//        Serial.print("Received Pitch: ");
//      }
//      //Serial.write(app_data.c_str());
//      Serial.println(app_data);
//    }
//  }

  targetAngle = fwd * 0.05 + zero_targetAngle;


}

void ps3_setup(void) {
  //    Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("8c:7c:b5:97:58:48");

  Serial.println("Ready.");
}

void onConnect() {
  Serial.println("Connected.");
}
