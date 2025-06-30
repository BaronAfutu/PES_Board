#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DCMotor.h"
#include "DebounceIn.h"
#include "FastPWM.h"
#include "IRSensor.h"
#include "Servo.h"

bool do_execute_main_task = false;
// this variable will be toggled via the user button (blue button) and decides
// whether to execute the main task or not

bool do_reset_all_once = false;
// this variable is used to reset certain variables and objects and
// shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);
// create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed
                                   // when user button gets pressed, definition
                                   // below

// set up states for state machine
enum RobotState {
  INITIAL,
  SLEEP,
  FORWARD,
  BACKWARD,
  EMERGENCY
} robot_state = RobotState::INITIAL;

// main runs as an own thread
int main() {

  // attach button fall function address to user button object
  user_button.fall(&toggle_do_execute_main_fcn);

  // while loop gets executed every main_task_period_ms milliseconds, this is a
  // simple approach to repeatedly execute main
  const int main_task_period_ms = 20;
  // define main task period time in ms e.g. 20 ms, therefore
  // the main task will run 50 times per second

  Timer main_task_timer; // create Timer object which we use to run the main
                         // task every main_task_period_ms

  // led on nucleo board
  DigitalOut user_led(LED1);

  // additional led
  // create DigitalOut object to command extra led, you need to add an
  // additional resistor, e.g. 220...500 Ohm a led has an anode (+) and a
  // cathode (-), the cathode needs to be connected to ground via the resistor
  DigitalOut led1(PB_9);

  /************************** IR SENSOR START **************************/
  IRSensor ir_sensor(PC_2);
  ir_sensor.setCalibration(8042.5280f, -274.2508f);

  // ir distance sensor
  float ir_distance_cm = 0.0f;
  /************************** IR SENSOR END ****************************/

  /*************************** SERVO MOTORS START **********************/
  // servo
  Servo servo_D0(PB_D0); // Reely
  Servo servo_D2(PB_D2); // Futuba

  // reely S0090
  float servo_D0_ang_min = 0.0350f;
  float servo_D0_ang_max = 0.1250f;

  // futuba S3001
  float servo_D2_ang_min = 0.0150f;
  float servo_D2_ang_max = 0.1250f;

  // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
  // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min,
  // servo_D0_ang_max)
  servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
  servo_D2.calibratePulseMinMax(servo_D2_ang_min, servo_D2_ang_max);

  // enable smooth movement
  // default acceleration of the servo motion profile is 1.0e6f
  // servo_D0.setMaxAcceleration(0.5f);
  // servo_D2.setMaxAcceleration(0.5f);

  float servo_input = 0.0f;
  int servo_counter = 0; // define servo counter, this is an additional variable
                         // used to command the servo
  const int loops_per_seconds = static_cast<int>(
      ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

  /***************************** SERVO MOTORS END ****************************/

  // mechanical button
  DigitalIn mechanical_button(PC_5);
  // create DigitalIn object to evaluate mechanical button, you
  // need to specify the mode for proper usage, see below
  mechanical_button.mode(PullUp); // sets pullup between pin and 3.3 V, so that
                                  // there is a defined potential

  /************************ DC MOTORS START ***********************************/
  // create object to enable power electronics for the DC motors
  DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

  // motor M1
  //   FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1

  const float voltage_max = 12.0f;
  // maximum voltage of battery packs, adjust this to
  // 6.0f V if you only use one battery pack

  /*
  // motor M2
  const float gear_ratio_M2 = 100.0f; // gear ratio
  const float kn_M2 = 140.0f / 12.0f; // motor constant [rpm/V]
  // it is assumed that only one motor is available, therefore
  // we use the pins from M1, so you can leave it connected to M1
  DCMotor motor_M2(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M2, kn_M2,
                   voltage_max);
  // limit max. acceleration to half of the default acceleration
  motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);
  // enable the motion planner for smooth movements
  //   motor_M2.enableMotionPlanner();

//   motor_M2.setVelocity(motor_M2.getMaxVelocity() * -0.5f);

  // limit max. velocity to half physical possible velocity
  motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 1.0f);
  */

  // motor M3
  const float gear_ratio_M3 = 100.0f; // gear ratio
  const float kn_M3 = 140.0f / 12.0f; // motor constant [rpm/V]
  // it is assumed that only one motor is available, therefore
  // we use the pins from M1, so you can leave it connected to M1
  DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M3, kn_M3,
                   voltage_max);
  // enable the motion planner for smooth movement
  motor_M3.enableMotionPlanner();
  // limit max. acceleration to half of the default acceleration
  motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);

  /***************************** DC MOTORS END *****************************/

  // start timer
  main_task_timer.start();

  // this loop will run forever
  while (true) {
    main_task_timer.reset();

    // print to the serial terminal
    // printf("US distance cm: %f \n", ir_distance_cm);

    if (do_execute_main_task) {

      // visual feedback that the main task is executed, setting this once would
      // actually be enough
      led1 = 1;

      // read us sensor distance, only valid measurements will update
      // us_distance_cm
      const float us_distance_cm_candidate = ir_sensor.read();
      if (us_distance_cm_candidate > 0.0f)
        ir_distance_cm = us_distance_cm_candidate;
      // min and max ultra sonic sensor reading, (ir_distance_min,
      // ir_distance_max) -> (servo_min, servo_max)
      float ir_distance_min = 3.0f;
      float ir_distance_max = 40.0f;

      // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
      //   motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5f);

      //   motor_M3.setRotation(3.0f);

      // print to the serial terminal
      //   printf("Motor velocity: %f \n", motor_M2.getVelocity());

      // print to the serial terminal
      //   printf("Motor position: %f \n", motor_M3.getRotation());

      // state machine
      switch (robot_state) {
      case RobotState::INITIAL: {
        // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
        enable_motors = 1;
        robot_state = RobotState::SLEEP;

        break;
      }
      case RobotState::FORWARD: {
        // press is moving forward until it reaches 2.9f rotations,
        // when reaching the value go to BACKWARD
        motor_M3.setRotation(2.9f);
        // setting this once would actually be enough
        // if the distance from the sensor is less than 4.5f cm,
        // we transition to the EMERGENCY state
        if (ir_distance_cm < 4.5f)
          robot_state = RobotState::EMERGENCY;
        // switching condition is slightly smaller for robustness
        if (motor_M3.getRotation() > 2.89f)
          robot_state = RobotState::BACKWARD;

        break;
      }
      case RobotState::SLEEP: {
        // wait for the signal from the user, so to run the process
        // that is triggered by clicking the mechanical button
        // then go to the FORWARD state
        if (mechanical_button.read())
          robot_state = RobotState::FORWARD;

        break;
      }
      case RobotState::EMERGENCY: {
        // disable the motion planner and
        // move to the initial position asap
        // then reset the system
        motor_M3.disableMotionPlanner();
        motor_M3.setRotation(0.0f);
        if (motor_M3.getRotation() < 0.01f)
          toggle_do_execute_main_fcn();

        break;
      }
      case RobotState::BACKWARD: {
        // move backwards to the initial position
        // and go to the SLEEP state if reached
        motor_M3.setRotation(0.0f);
        // switching condition is slightly bigger for robustness
        if (motor_M3.getRotation() < 0.01f)
          robot_state = RobotState::SLEEP;

        break;
      }
      default: {

        break; // do nothing
      }
      }

      // print to the serial terminal
      printf("US Sensor in cm: %f, DC Motor Rotations: %f\n", ir_distance_cm,
             motor_M3.getRotation());

    } else {
      // the following code block gets executed only once
      if (do_reset_all_once) {
        do_reset_all_once = false;

        // reset variables and objects
        led1 = 0;
        enable_motors = 0;
        ir_distance_cm = 0.0f;
        motor_M3.setMotionPlanerPosition(0.0f);
        motor_M3.setMotionPlanerVelocity(0.0f);
        motor_M3.enableMotionPlanner();
        robot_state = RobotState::INITIAL;
      }

      //   motor_M2.setVelocity(motor_M2.getMaxVelocity() * -0.5f);
      //   motor_M3.setRotation(-3.0f);
    }

    // toggling the user led
    user_led = !user_led;

    // read timer and make the main thread sleep for the remaining time span
    // (non blocking)
    int main_task_elapsed_time_ms =
        duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
    if (main_task_period_ms - main_task_elapsed_time_ms < 0)
      printf("Warning: Main task took longer than main_task_period_ms\n");
    else
      thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
  }
}

void toggle_do_execute_main_fcn() {
  // toggle do_execute_main_task if the button was pressed
  do_execute_main_task = !do_execute_main_task;
  // set do_reset_all_once to true if do_execute_main_task changed from false to
  // true
  if (do_execute_main_task)
    do_reset_all_once = true;
}