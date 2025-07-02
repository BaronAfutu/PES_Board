#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IMU.h"
#include "Servo.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
  // attach button fall function address to user button object
  user_button.fall(&toggle_do_execute_main_fcn);

  // while loop gets executed every main_task_period_ms milliseconds, this is a
  // simple approach to repeatedly execute main
  const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                      // the main task will run 50 times per second
  Timer main_task_timer;              // create Timer object which we use to run the main task
                                      // every main_task_period_ms

  // led on nucleo board
  DigitalOut user_led(LED1);

  // --- adding variables and objects and applying functions starts here ---

  // imu
  ImuData imu_data;
  IMU imu(PB_IMU_SDA, PB_IMU_SCL);
  Eigen::Vector2f rp(0.0f, 0.0f);

  // minimal pulse width and maximal pulse width obtained from the servo calibration process
  // modelcraft RS2 MG/BB
  float servo_ang_min = 0.035f;
  float servo_ang_max = 0.130f;

  // angle limits of the servos
  const float angle_range_min = -M_PIf / 2.0f;
  const float angle_range_max = M_PIf / 2.0f;

  // angle to pulse width coefficients
  const float normalised_angle_gain = 1.0f / M_PIf;
  const float normalised_angle_offset = 0.5f;

  // pulse width
  static float roll_servo_width = 0.5f;
  static float pitch_servo_width = 0.5f;

  // start timer
  main_task_timer.start();

  // this loop will run forever
  while (true)
  {
    main_task_timer.reset();

    if (do_execute_main_task)
    {

      // --- code that runs when the blue button was pressed goes here ---

      // read imu data
      imu_data = imu.getImuData();
      // acceleration in meters per second squared in three axes
      float acc_x = imu_data.acc(0);
      float acc_y = imu_data.acc(1);
      float acc_z = imu_data.acc(2);

      // gyroscope in radians per second in three axes
      float gyro_x = imu_data.gyro(0);
      float gyro_y = imu_data.gyro(1);
      float gyro_z = imu_data.gyro(2);

      // Ours uses this one
      // roll, pitch, yaw according to Tait-Bryan angles ZYX
      // where R = Rz(yaw) * Ry(pitch) * Rx(roll) for ZYX sequence
      // singularity at pitch = +/-pi/2 radians (+/- 90 deg)
      float roll = imu_data.rpy(0);
      float pitch = imu_data.rpy(1);
      float yaw = imu_data.rpy(2);

      // // pitch, roll, yaw according to Tait-Bryan angles ZXY
      // // where R = Rz(yaw) * Rx(roll) * Ry(pitch)
      // // singularity at roll = +/-pi/2
      // float pitch = imu_data.pry(0);
      // float roll = imu_data.pry(1);
      // float yaw = imu_data.pry(2);

      // Linear accelerations in x, y, z directions
      printf("Accelerations: acc_x=%.4f | acc_y=%.4f | acc_z=%.4f\n", acc_x, acc_y, acc_z);

      // Angular accelerations around x,y,z directions. Measures accelation during motion.
      // If no motion, no gyro value irrespective of angle of pry
      printf("Gyros gyro_x=%.4f | gyro_y=%.4f | gyro_z=%.4f\n", gyro_x, gyro_y, gyro_z);
      
      // The angle of orientation
      printf("Orientations pitch=%.4f | roll=%.4f | yaw=%.4f\n", pitch, roll, yaw);

      // thread_sleep_for(200);
    }
    else
    {
      // the following code block gets executed only once
      if (do_reset_all_once)
      {
        do_reset_all_once = false;

        // --- variables and objects that should be reset go here ---

        // reset variables and objects
        roll_servo_width = 0.5f;
        pitch_servo_width = 0.5f;
      }
    }

    // toggling the user led
    user_led = !user_led;

    // --- code that runs every cycle goes here ---

    // print to the serial terminal
    // printf("%6.2f, %6.2f \n", roll_servo_width, pitch_servo_width);

    // read timer and make the main thread sleep for the remaining time span (non blocking)
    int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
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
  // set do_reset_all_once to true if do_execute_main_task changed from false to true
  if (do_execute_main_task)
    do_reset_all_once = true;
}