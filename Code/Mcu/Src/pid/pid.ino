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

#define Kp  8500.0  //3000.0  //2700.0  // 2000 //Ku= 8000 tu=
#define Kd  6000.0    //9000.0    //9.0     // 20.0  
#define Ki  30000.0 //30000.0 //53000.0 //22000    
#define Kyaw 0.02
#define sampleTime  0.01  // 100 Hz
#define zero_targetAngle 0.025  // Calibrated point

uint8_t i;

float angles[3];
float targetAngle = 0.0;
float currentAngle = 0.0, prevAngle = 0.0, error = 0.0, prevError = 0.0, errorSum = 0.0, yaw = 0.0, targetYaw = 0.0;
int pwm = 0;
volatile bool controlFlag = false;

bool ps3_joy = true;
bool mit_joy = false;
#define yaw_ascii "Y"
#define pitch_ascii "P"
char in_byte;
String app_data = "";
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
  //imu_setup_dmp();

  // Bluetooth init
  //  Dabble.begin("MyEsp32");       //set bluetooth name of your device
  if (ps3_joy) {
    ps3_setup();
  }

  //Bluetooth Setup
  if (mit_joy) {
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

    if (ps3_joy) {
      x_down = Ps3.event.button_down.cross;
      if (x_down) {
        stop_command = abs(stop_command - 1);
      }
    }
    else {
      stop_command = false;
    }

    if (!stop_command) {

      // Complementary Filter
      getAccelGyro(&ax, &az, &gy, &gz);
      accelPitch = atan2(ax, az) * RAD_TO_DEG;
      pitch = (tau) * (pitch + (-gy/2) * sampleTime) + (1 - tau) * (accelPitch); //TODO: poruqe hace fata un /2 acá

      //low pass (exponential) filter
      currentAngle = pitch * DEG_TO_RAD*0.1 + currentAngle *0.9;

      // DMP
//      getEulerAngles_dmp(angles);
//      currentAngle = angles[2];
//      gy = RAD_TO_DEG * (currentAngle - prevAngle) / sampleTime;

      error = currentAngle - targetAngle;
      errorSum += error;
      errorSum = constrain(errorSum, -15, 15);
      pwm = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * gy * sampleTime * DEG_TO_RAD;
      
      if (pwm > -0) //to take into account the static friction of the motors
        pwm += 12;
      if (pwm < 0)
        pwm -= 12;

      pwm = constrain(pwm, -255, 255);

      prevAngle = currentAngle;

      yaw = yaw + gz * DEG_TO_RAD;
      rot = Kyaw * (yaw - targetYaw);

      if (currentAngle < 0.3 && currentAngle > -0.3 ) //when the robot falls we turn it off
      {
        L_motor(constrain(pwm + int(rot * 60), -255, 255));
        R_motor(constrain(pwm - int(rot * 60), -255, 255));
      }
      else
      {
        L_motor(0);
        R_motor(0);
      }

      if (true) {
//        Serial.print("fwd: ");
//        Serial.print(fwd);
//        Serial.print(" rot: ");
//        Serial.print(rot);
//        Serial.print("  Pitch:  ");
//        Serial.println(currentAngle, 4);
        Serial.print("  sum:");
        Serial.print(pwm);
        Serial.print("  p:");
        Serial.print(Kp * error);
        Serial.print("  i:");
        Serial.print(Ki * errorSum* sampleTime);
        Serial.print("  d:");
        Serial.println(Kd * gy * sampleTime * DEG_TO_RAD);
        //Serial.print("  Yaw:  ");
        //Serial.print(yaw, 4);
//        Serial.print("  Gy:  ");
//        Serial.print(gy, 4);
//        Serial.print("  PWM:  ");
//        Serial.println(pwm);
//        Serial.print("  Loop time: ");
//        time_ctr = micros() - time_ctr;
//        Serial.println(time_ctr);
//        time_ctr = micros();
        //        Serial.print("Received Yaw: ");
        //        Serial.println(app_data.toFloat() - 128);
        //        Serial.print("Received Pitch: ");
        //        Serial.println((app_data.toFloat() - 128) / 128);
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
    if (mit_joy) {
      if (SerialBT.available()) {
  
        in_byte = SerialBT.read();
  
        if (in_byte != '-') //would like to use /n as delimiter but dunno how to send non printables in app inventor
        {
          app_data += String(in_byte);
        }
        else
        {
          if (app_data.startsWith(yaw_ascii))
          {
            app_data.setCharAt(0, '0');
  
            targetYaw = (app_data.toFloat() - 128);
          }
  
          else if (app_data.startsWith(pitch_ascii))
          {
            app_data.setCharAt(0, '0');
  
            fwd = (app_data.toFloat() - 128) / 128;
          }
          app_data = "";
        }
      }
    }

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
