#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "IMU.h"
#include "SensorBar.h"
#include "Servo.h"
#include "IRSensor.h"
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
  const float r_wheel = 0.1f / 2.0f; // 0.035f / 2.0f;
  const float b_wheel = 0.160f;      // 0.157f;
  Eigen::Matrix2f Cwheel2robot;
  Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f,
      r_wheel / b_wheel, -r_wheel / b_wheel;

  Eigen::Vector2f robot_coord = {0.0f, 0.0f};
  Eigen::Vector2f wheel_speed = {0.0f, 0.0f};

  /********** SENSOR BAR SETUP **********/
  const float bar_dist = 0.095f; //;0.114f; // Distance from wheel axis to sensorbar
  SensorBar sensor_bar(PB_9, PB_8, bar_dist);
  float angle = 0.0f;

  /********** CONTROLLER TUNING **********/
  const float Kp{7.0f}; // base gain
  const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
  const float v_max = 0.5f * wheel_vel_max * r_wheel; // maximum forward speed 0.9
  const float gamma = 4.0f;                           // speed reduction factor 2
  const float v_min = 0.05f;                          // minimum forward speed (m/s)
  const float angle_exp = 1.0f;                       // exponent for nonlinear steering
  float v = 0.0f, omega = 0.0f;

  /********** Ball Balancing ************/
  // servos
  Servo servo_roll(PB_D1);
  Servo servo_pitch(PB_D2);
  Servo servo_lift(PB_D0);

  // imu
  ImuData imu_data;
  IMU imu(PB_IMU_SDA, PB_IMU_SCL);
  Eigen::Vector2f rp(0.0f, 0.0f);

  // minimal pulse width and maximal pulse width obtained from the servo calibration process
  // modelcraft RS2 MG/BB
  float servo_ang_min = 0.035f;
  float servo_ang_max = 0.125;

  // futuba S3001
  float servo_lift_ang_min = 0.0150f;
  float servo_lift_ang_max = 0.1250f;

  // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
  // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
  servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
  servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);
  servo_lift.calibratePulseMinMax(servo_lift_ang_min, servo_lift_ang_max);

  // angle limits of the servos
  const float angle_range_min = -M_PIf / 2.0f;
  const float angle_range_max = M_PIf / 2.0f;

  // angle to pulse width coefficients
  const float normalised_angle_gain = 1.0f / M_PIf;
  const float normalised_angle_offset = 0.5f;

  // pulse width
  static float roll_servo_width = 0.5f;
  static float pitch_servo_width = 0.5f;
  static float lift_servo_width = 0.68f;

  servo_roll.setPulseWidth(roll_servo_width);
  servo_pitch.setPulseWidth(pitch_servo_width);
  servo_lift.setPulseWidth(lift_servo_width);

  // linear 1-D mahony filter
  const float Ts = static_cast<float>(main_task_period_ms) * 1.0e-3f; // sample time in seconds
  const float kp = 3.0f;
  //   const float kp = 8.0f;
  float roll_estimate = 0.0f;
  float pitch_estimate = 0.0f;

  /****** IR Sensor **********/
  IRSensor ir_sensor(PC_2);
  ir_sensor.setCalibration(8042.5280f, -274.2508f);
  // ir distance sensor
  const float ir_dist_thresh = 4.2f; // define a variable to store measurement (in mV)
  float ir_distance_cm = 0.0f;
  int lift_up_counter = 0;
  bool isLiftedUp = false;

  main_task_timer.start();

  while (true)
  {
    main_task_timer.reset();

    // read imu data
    imu_data = imu.getImuData();
    // linear 1-D mahony filter
    const float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2));   // roll angle from accelerometer
    const float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2)); // pitch angle from accelerometer
    roll_estimate += Ts * (imu_data.gyro(0) + kp * (roll_acc - roll_estimate));
    pitch_estimate += Ts * (imu_data.gyro(1) + kp * (pitch_acc - pitch_estimate));
    rp(0) = roll_estimate;  // roll angle
    rp(1) = pitch_estimate; // pitch angle

    printf("IR distance cm: %f \n", ir_distance_cm);
    if (do_execute_main_task)
    {
      led1 = 1;
      enable_motors = 0;

      if (sensor_bar.isAnyLedActive())
        angle = sensor_bar.getAvgAngleRad();

      // read analog input
      ir_distance_cm = ir_sensor.read();
      // min and max ultra sonic sensor reading, (ir_distance_min,
      // ir_distance_max) -> (servo_min, servo_max)
      float ir_distance_min = 3.0f;
      float ir_distance_max = 40.0f;
      float ir_distance_treshhold = 20.0f;

      // enable the servos
      if (!servo_roll.isEnabled())
        servo_roll.enable();
      if (!servo_pitch.isEnabled())
        servo_pitch.enable();
      if (!servo_lift.isEnabled())
        servo_lift.enable();

      // map to servo commands
      roll_servo_width = -normalised_angle_gain * rp(0) + (normalised_angle_offset + 0.1);
      pitch_servo_width = normalised_angle_gain * rp(1) + (normalised_angle_offset + 0.35);
      if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
        servo_roll.setPulseWidth(roll_servo_width);
      if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
        servo_pitch.setPulseWidth(pitch_servo_width);

      printf("%f\n", pitch_servo_width);

      if (ir_distance_cm <= ir_dist_thresh)
      {
        isLiftedUp = true;
      }

      if (!isLiftedUp)
      {
        servo_lift.setPulseWidth(lift_servo_width);

        // float omega = Kp * std::pow(angle, angle_exp);
        omega = Kp * angle * std::fabs(angle);

        // Forward speed reduction: v_max / (1 + gamma * |angle|)
        v = v_max / (1.0f + gamma * std::fabs(angle));
        v = std::max(v, v_min);
      }
      else
      { // lift the Line follower array up and drive straight
        servo_lift.setPulseWidth(lift_servo_width - 0.5);
        v = v_max;
        omega = 0;

        lift_up_counter++;
        if (lift_up_counter >= 1 * 50)
        {
          lift_up_counter = 0;
          isLiftedUp = false;
        }
      }
      // servo_lift.setPulseWidth(lift_servo_width + ((pitch_servo_width - 0.887) * -2.0f));

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
