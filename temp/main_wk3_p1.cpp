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
  EXECUTION,
  SLEEP,
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
  // limit max. velocity to half physical possible velocity
  motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);

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

      // read analog input
      ir_distance_cm = ir_sensor.read();
      // min and max ultra sonic sensor reading, (ir_distance_min,
      // ir_distance_max) -> (servo_min, servo_max)
      float ir_distance_min = 3.0f;
      float ir_distance_max = 40.0f;

      enable_motors = 1;

      //   pwm_M1.write((1.0f * (36.0 / 7.0) + 12) / 24.0); // apply 6V to the

      // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
      //   motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5f);

      motor_M3.setRotation(3.0f);

      // print to the serial terminal
      //   printf("Motor velocity: %f \n", motor_M2.getVelocity());

      // print to the serial terminal
      printf("Motor position: %f \n", motor_M3.getRotation());

      /*
      // state machine
      switch (robot_state) {
      case RobotState::INITIAL: {
        // printf("INITIAL\n");
        // enable the servo
        if (!servo_D0.isEnabled())
          servo_D0.enable();
        robot_state = RobotState::EXECUTION;

        break;
      }
      case RobotState::EXECUTION: {
        // printf("EXECUTION\n");
        // function to map the distance to the servo movement (ir_distance_min,
        // ir_distance_max) -> (0.0f, 1.0f)
        servo_input = (ir_distance_cm - ir_distance_min) /
                      (ir_distance_max - ir_distance_min);
        // values smaller than 0.0f or bigger than 1.0f are constrained to the
        // range (0.0f, 1.0f) in setPulseWidth
        servo_D0.setPulseWidth(servo_input);

        // if the measurement is outside the min or max limit go to SLEEP
        if ((ir_distance_cm < ir_distance_min) ||
            (ir_distance_cm > ir_distance_max))
          robot_state = RobotState::SLEEP;

        // if the mechanical button is pressed go to EMERGENCY
        if (mechanical_button.read())
          robot_state = RobotState::EMERGENCY;

        break;
      }
      case RobotState::SLEEP: {
        // printf("SLEEP\n");
        // if the measurement is within the min and max limits go to EXECUTION
        if ((ir_distance_cm > ir_distance_min) &&
            (ir_distance_cm < ir_distance_max))
          robot_state = RobotState::EXECUTION;

        // if the mechanical button is pressed go to EMERGENCY
        if (mechanical_button.read())
          robot_state = RobotState::EMERGENCY;

        break;
      }
      case RobotState::EMERGENCY: {
        // printf("EMERGENCY\n");
        // the transition to the emergency state causes the execution of the
        // commands contained in the outer else statement scope, and since
        // do_reset_all_once is true the system undergoes a reset
        toggle_do_execute_main_fcn();

        break;
      }
      default: {

        break; // do nothing
      }
      }*/

    } else {
      // the following code block gets executed only once
      if (do_reset_all_once) {
        do_reset_all_once = false;

        // reset variables and objects
        led1 = 0;
        servo_D0.disable();
        ir_distance_cm = 0.0f;
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