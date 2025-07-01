#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "SensorBar.h"
#include <Eigen/Dense>
#include <cmath>

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false;
bool do_reset_all_once = false;

// user button handling
DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
  user_button.fall(&toggle_do_execute_main_fcn);

  const int main_task_period_ms = 20;
  Timer main_task_timer;
  DigitalOut user_led(LED1);
  DigitalOut led1(PB_9);

  /********** DC MOTORS SETUP **********/
  DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
  const float voltage_max = 12.0f;
  const float gear_ratio = 100.0f;
  const float kn = 140.0f / 12.0f;
  DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
  DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

  // kinematic transform from wheel speeds to robot v,ω
  const float r_wheel = 0.035f / 2.0f;
  const float b_wheel = 0.157f;
  Eigen::Matrix2f Cwheel2robot;
  Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f,
      r_wheel / b_wheel, -r_wheel / b_wheel;

  Eigen::Vector2f robot_coord = {0.0f, 0.0f};
  Eigen::Vector2f wheel_speed = {0.0f, 0.0f};

  /********** SENSOR BAR SETUP **********/
  const float bar_dist = 0.114f;
  SensorBar sensor_bar(PB_9, PB_8, bar_dist);
  float angle = 0.0f;

  /********** CONTROLLER TUNING **********/
  const float Kp{7.0f}; // base gain
  const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
  const float v_max = 1.0f * wheel_vel_max * r_wheel; // maximum forward speed 0.9
  const float gamma = 3.0f;                           // speed reduction factor 2
  const float v_min = 0.05f;                          // minimum forward speed (m/s)
  const float angle_exp = 1.0f;                       // exponent for nonlinear steering

  main_task_timer.start();

  while (true)
  {
    main_task_timer.reset();

    if (do_execute_main_task)
    {
      led1 = 1;
      enable_motors = 1;

      if (sensor_bar.isAnyLedActive())
        angle = sensor_bar.getAvgAngleRad();

      // Nonlinear steering: cubic in angle
      float omega = Kp * std::pow(angle, angle_exp);

      // Forward speed reduction: v_max / (1 + gamma * |angle|)
      float v = v_max / (1.0f + gamma * std::fabs(angle));
      v = std::max(v, v_min);

      // Pack into robot_coord (v, omega)
      robot_coord = {v, omega};

      // Convert to wheel speeds [rad/s]
      wheel_speed = Cwheel2robot.inverse() * robot_coord;

      // Send to motors (convert rad/s to rps)
      motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
      motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

      printf("Angle: %0.3f  v: %0.3f  ω: %0.3f\n", angle, v, omega);
    }
    else
    {
      if (do_reset_all_once)
      {
        do_reset_all_once = false;
        led1 = 0;
        enable_motors = 0;
      }
    }

    user_led = !user_led;

    // maintain loop rate
    int main_task_elapsed_time_ms =
        duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
    if (main_task_period_ms - main_task_elapsed_time_ms < 0)
      printf("Warning: Main task took longer than main_task_period_ms\n");
    else
      thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
  }
}

void toggle_do_execute_main_fcn()
{
  // toggle do_execute_main_task if the button was pressed
  do_execute_main_task = !do_execute_main_task;
  // set do_reset_all_once to true if do_execute_main_task changed from false to
  // true
  if (do_execute_main_task)
    do_reset_all_once = true;
}
