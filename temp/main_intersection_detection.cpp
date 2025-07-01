#include "mbed.h"
// Code for intersection detection

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "FastPWM.h"
#include "SensorBar.h"
#include "LineFollower.h"
#include <Eigen/Dense>
#include <cstdlib>

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
  user_button.fall(&toggle_do_execute_main_fcn);
  const int main_task_period_ms = 20;
  Timer main_task_timer;
  DigitalOut user_led(LED1);
  DigitalOut led1(PB_9);

  // DC Motors setup
  DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
  const float voltage_max = 12.0f;
  const float gear_ratio = 100.00f;
  const float kn = 140.0f / 12.0f;
  DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
  DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

  const float r_wheel = 0.035f / 2.0f;
  const float b_wheel = 0.157f;
  Eigen::Matrix2f Cwheel2robot;
  Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f,
      r_wheel / b_wheel, -r_wheel / b_wheel;
  Eigen::Vector2f robot_coord = {0.0f, 0.0f};
  Eigen::Vector2f wheel_speed = {0.0f, 0.0f};

  // Sensor Bar
  const float bar_dist = 0.114f;
  SensorBar sensor_bar(PB_9, PB_8, bar_dist);
  float angle = 0.0f;

  // Controller tuning
  const float Kp = 5.0f;
  const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

  main_task_timer.start();
  srand(0);

  while (true)
  {
    main_task_timer.reset();

    if (do_execute_main_task)
    {
      led1 = 1;
      enable_motors = 1;

      if (sensor_bar.isAnyLedActive())
        angle = sensor_bar.getAvgAngleRad();

      // Intersection and end detection
      float leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
      float centerMean = sensor_bar.getMeanFourAvgBitsCenter();
      float rightMean = sensor_bar.getMeanThreeAvgBitsRight();

      // End of maze: all three detect line
      bool isEnd = (leftMean < 0.5f && centerMean > 0.5f && rightMean < 0.5f) ||
                   (centerMean > 0.7f && !sensor_bar.getAvgBit(0) && !sensor_bar.getAvgBit(7));
      if (isEnd)
      {
        motor_M1.setVelocity(0);
        motor_M2.setVelocity(0);
        for (int i = 0; i < 3; ++i)
        {
          led1 = 1;
          thread_sleep_for(200);
          led1 = 0;
          thread_sleep_for(200);
        }
        do_execute_main_task = false;
        continue;
      }

      bool leftBranch = (leftMean > 0.5f && centerMean > 0.5f && rightMean <= 0.5f && sensor_bar.getAvgBit(7)==1);
      bool rightBranch = (rightMean > 0.5f && centerMean > 0.5f && leftMean <= 0.5f && sensor_bar.getAvgBit(0)==1);
      bool crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
      bool isIntersection = (leftBranch || rightBranch || crossRoad);

      if (isIntersection)
      {
        motor_M1.setVelocity(0);
        motor_M2.setVelocity(0);
        for (int i = 0; i < 2; ++i)
        {
          led1 = 1;
          thread_sleep_for(200);
          led1 = 0;
          thread_sleep_for(200);
        }
        int dir = 0;
        if (crossRoad)
          dir = (rand() & 1) ? 1 : -1;
        else if (leftBranch)
          dir = -1;
        else
          dir = 1;
        float turn_speed = Kp * dir * (M_PIf / 2.0f);
        robot_coord = {0.0f, turn_speed};
        wheel_speed = Cwheel2robot.inverse() * robot_coord;
        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));
        thread_sleep_for(500);
        continue;
      }

      // Normal line-following
      robot_coord = {0.5f * wheel_vel_max * r_wheel, Kp * angle};
      wheel_speed = Cwheel2robot.inverse() * robot_coord;
      motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
      motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

      printf("Angle: %0.2f  L:%0.2f C:%0.2f R:%0.2f\n", angle, leftMean, centerMean, rightMean);
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
    int elapsed = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
    if (main_task_period_ms - elapsed > 0)
      thread_sleep_for(main_task_period_ms - elapsed);
  }
}

void toggle_do_execute_main_fcn()
{
  do_execute_main_task = !do_execute_main_task;
  if (do_execute_main_task)
    do_reset_all_once = true;
}
