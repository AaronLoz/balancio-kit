#include "dc_motor.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"

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

float targetAngle = 0.0;
float currentAngle = 0.0, prevAngle = 0.0, error = 0.0, prevError = 0.0, errorSum = 0.0, yaw = 0.0, targetYaw = 0.0;
int pwm = 0;
volatile bool controlFlag = false;

float fwd = 0;
float rot = 0;

unsigned long pid_count = 0;

int time_ctr = 0;
float ax, az, gy , gz;
float accelPitch, pitch;
float tau = 0.98;

bool stop_command = true;
bool x_down;

void setup() {
  // UART PC connection
  Serial.begin(115200);

  // Motor initialization
  motor_init();

  // IMU init
  imu_setup();

  // Bluetooth init
  //  Dabble.begin("MyEsp32");       //set bluetooth name of your device
  ps3_setup();

  // Timer init. (ISR at 100 Hz)
  timer_init();
}

// ISR at 100 Hz
void IRAM_ATTR onTime() {
  controlFlag = true;
}

void loop() {

  if (controlFlag) {

    x_down = Ps3.event.button_down.cross;
    if (x_down) {
      stop_command = abs(stop_command - 1);
    }
    stop_command = false;

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

      if (true) {
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


  fwd = -Ps3.data.analog.stick.ry / 128.0;
  //rot = Ps3.data.analog.stick.lx / 128.0;
  
  targetYaw += Ps3.data.analog.stick.lx / 50.0;

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
