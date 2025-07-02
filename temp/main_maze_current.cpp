#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include "LineFollower.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();
float getrevolutions(float deg_angle, float wr, float wb);
bool isIntersection(float, float, float, bool *, bool *, bool *);

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
    // the main task will run 50 times per second
    Timer main_task_timer;

    // led on nucleo board
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    /* ***************** DC Motors setup *********************/
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    const float voltage_max = 12.0f;
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); // ? Right or Left
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); // ? Right or Left

    const float r_wheel = 0.035f / 2.0f;
    const float b_wheel = 0.157f;
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f,
        r_wheel / b_wheel, -r_wheel / b_wheel;
    Eigen::Vector2f robot_coord = {0.0f, 0.0f};
    Eigen::Vector2f wheel_speed = {0.0f, 0.0f};

    // Sensor Bar
    const float bar_dist = 0.114f; // Distance from wheel axis to sensorbar
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);
    float angle = 0.0f;
    float leftMean = 0.0f, centerMean = 0.0f, rightMean = 0.0f;

    // Controller tuning // TODO Use the non-linear controller later
    const float Kp = 5.0f;
    const float wheel_vel_max = 1.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    float v = 0.5f * wheel_vel_max * r_wheel;
    float w = Kp * angle;

    // navigation variables
    bool leftTurn = false, rightTurn = false, crossRoad = false, intersection = false;
    int dir = 0;

    // set up states for state machine
    enum RobotState
    {
        INITIAL,
        SLEEP,
        TRAIN,
        RUN,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    enum MotionState
    {
        FOLLOWLINE,
        TURN,
        UTURN
    } motion_state = MotionState::FOLLOWLINE;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true)
    {
        main_task_timer.reset();

        if (do_execute_main_task)
        {

            // --- code that runs when the blue button was pressed goes here ---
            switch (robot_state)
            {
            case RobotState::INITIAL:
            {
                // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                enable_motors = 1;
                robot_state = RobotState::TRAIN;

                break;
            }
            case RobotState::TRAIN:
            {
                switch (motion_state)
                {
                case MotionState::FOLLOWLINE:
                    // Normal line-following

                    // Controller implementation
                    v = 0.5f * wheel_vel_max * r_wheel;
                    w = Kp * angle;
                    robot_coord = {v, w};
                    wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

                    leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
                    centerMean = sensor_bar.getMeanFourAvgBitsCenter();
                    rightMean = sensor_bar.getMeanThreeAvgBitsRight();
                    if (isIntersection(leftMean, centerMean, rightMean, &leftTurn, &rightTurn, &crossRoad)){
                        motion_state = MotionState::TURN;
                    }
                        break;
                default:
                    break;
                }
            }
            }

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        }
        else
        {
            // the following code block gets executed only once
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle goes here ---

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
float getrevolutions(float deg_angle, float wr, float wb)
{
    float L = (deg_angle / 360.0f) * 2.0f * M_PIf * wb;
    return L / (2 * M_PIf * wr);
}

bool isIntersection(float leftMean, float centerMean, float rightMean, bool *leftTurn, bool *rightTurn, bool *crossRoad)
{
    *leftTurn = (leftMean > 0.5f && centerMean > 0.3f && rightMean <= 0.5f);
    *rightTurn = (rightMean > 0.5f && centerMean > 0.3f && leftMean <= 0.5f);
    *crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
    bool isIntersection = (leftTurn || rightTurn || crossRoad);
    return isIntersection;
}