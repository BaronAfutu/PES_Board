/**
 * Course: Mech 617 - Applied Systems Engineering
 * Group 1- Truck Titans
 * Authors: Baron Afutu
 *          Tabitha Njoki
 *          Patrick Akwasi Nyankamango
 *          Reginald Sai-Obodai
 *
 * Description:
 *  Complete code for solving the maze using state machines
 *  This solved the maze for robot prototype 2 as r_wheel
 *      and b_wheel indicate
 *  Uses Wall Following Algorithm with Right Hand Rule to
 *      explore and solve any aribituary maze
 *  Uses differential drive kinematics to initiate turns
 *  This is not the final code
 */

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();
/**
 * @brief Calculates the number of wheel revolutions required for a differential-drive robot to turn a given angle.
 *
 * @param deg_angle  Turn angle in degrees (positive = counterclockwise, negative = clockwise).
 * @param wr         Wheel radius (meters).
 * @param wb         Wheelbase length (distance between wheels, in same units as `wr`).
 * @return float     Number of wheel revolutions needed to complete the turn.
 */
float getrevolutions(float deg_angle, float wr, float wb);

/**
 * @brief Stores a movement direction in the path array and simplifies the path if possible.
 *
 * @param direction    The direction to store ('L', 'R', 'S', 'B' for Left/Right/Straight/Back).
 * @param pathLength   Current length of the path (updated after storing the new direction).
 * @param path         Fixed-size array storing the path history (max 100 elements).
 *
 * @details
 * 1. Appends the new direction to the `path` array and increments `pathLength`.
 * 2. Prints the current path (for debugging).
 * 3. Calls `simplifyPath()` to optimize the path (e.g., remove redundant turns).
 */
void storePath(char &direction, unsigned char &pathLength, char (&path)[100]);
void simplifyPath(char (&path)[100], unsigned char &pathLength);

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
    // the main task will run 50 times per second
    int counter = 0; // variable used to track time as some task would run for specific periods
    Timer main_task_timer;

    // led on nucleo board
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);

    // DC Motors setup
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    const float voltage_max = 12.0f;
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); // Right
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); // Left

    const float r_wheel = 0.11f / 2.0f; // 0.035f / 2.0f;
    const float b_wheel = 0.160f;       // 0.157f;
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f,
        r_wheel / b_wheel, -r_wheel / b_wheel;
    Eigen::Vector2f robot_coord = {0.0f, 0.0f};
    Eigen::Vector2f wheel_speed = {0.0f, 0.0f};

    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f);
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);

    // Sensor Bar
    const float bar_dist = 0.095f; //;0.114f; // Distance from wheel axis to sensorbar
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);
    float angle = 0.0f;
    float leftMean = 0.0f, centerMean = 0.0f, rightMean = 0.0f;

    // Controller
    const float Kp = 6.0f;
    const float wheel_vel_max = 1.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    float v = 0.3f * wheel_vel_max * r_wheel;
    float w = Kp * angle;

    // navigation variables
    bool leftTurn = false, rightTurn = false, crossRoad = false, isIntersection = false, isDeadEnd = false;
    char dir = '0';

    float finalAngle = 0; // the final angle after which the moving wheel can completed a turn.
    float fixedAngle = 0; // the angle the fixed wheel should remain at when completing a turn

    char path[100] = " ";
    unsigned char pathLength = 0; // length of the path and index used while training. char is used in place of uint_8.
    int pathIndex = 0;            // Index used after the robot is done training and now in speedrun
    unsigned int status = 0;      // exploring and solving = 0; speed run = 1

    int deadEndCounter = 0; // counter used to confirm that a dead end has actually been reached.

    // set up states for state machine
    enum RobotState
    {
        INITIAL,
        SLEEP,
        TRAIN,
        RUN
    } robot_state = RobotState::INITIAL;

    enum MotionState
    {
        FOLLOWLINE,
        PREPARE_TURN, // Robot moves forward before initiating a curve to check if there is a straight line ahead
        LEFT_TURN,
        RIGHT_TURN,
        UTURN,
        STOP
    } motion_state = MotionState::FOLLOWLINE;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true)
    {
        main_task_timer.reset();

        if (counter % 50 == 0) // printed every second
        {
            printf("Robot state: %d, Motion State: %d, counter: %d\n", robot_state, motion_state, counter);
            counter = 0;
        }

        if (do_execute_main_task)
        {
            led1 = 1;
            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();

            leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
            centerMean = sensor_bar.getMeanFourAvgBitsCenter();
            rightMean = sensor_bar.getMeanThreeAvgBitsRight();

            switch (robot_state)
            {
            case RobotState::INITIAL:
            {
                enable_motors = 1;
                if (status == 0)
                {
                    robot_state = RobotState::TRAIN;
                    motion_state = MotionState::FOLLOWLINE;
                }
                else if (status == 1)
                {
                    robot_state = RobotState::RUN;
                    motion_state = MotionState::FOLLOWLINE;
                }
                break;
            }
            case RobotState::SLEEP:
            {
                enable_motors = 0;
                if (do_execute_main_task)
                {
                    do_execute_main_task = false;
                    do_reset_all_once = true;
                    robot_state = RobotState::INITIAL;
                    motion_state = MotionState::FOLLOWLINE;
                }
                break;
            }
            case RobotState::TRAIN:
            {
                switch (motion_state)
                {
                case MotionState::FOLLOWLINE:
                {

                    // Intersection and end detection
                    leftTurn = (leftMean > 0.75f && centerMean > 0.5f && rightMean <= 0.5f);
                    rightTurn = (rightMean > 0.75f && centerMean > 0.5f && leftMean <= 0.5f);
                    crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
                    isDeadEnd = (leftMean < 0.1f && centerMean < 0.1f && rightMean < 0.1f);
                    isIntersection = (leftTurn || rightTurn || crossRoad);

                    if (isIntersection)
                    {
                        if (crossRoad)
                        {
                            dir = 'R'; // using Right Hand Rule
                        }
                        else if (leftTurn)
                            dir = 'L';
                        else
                            dir = 'R';

                        // Robot needs to move forward 35-40mm to check if there is a straight line ahead of an intersection
                        // 0.125 is the number of rotation required of 35-40mm given wheel radius
                        // 0.125 = 40mm/(2*pi*r_wheel)
                        fixedAngle = motor_M1.getRotation() + 0.125;
                        finalAngle = motor_M2.getRotation() + 0.125;
                        motion_state = MotionState::PREPARE_TURN;
                    }
                    else if (isDeadEnd)
                    {
                        deadEndCounter++;
                        if (deadEndCounter >= 1 * 50) // If not line is detected for 1 second (50 cycles=>1 second), it must be a dead end
                        {
                            // Preparing for an 180 degree inplace turn
                            fixedAngle = motor_M1.getRotation() + getrevolutions(-91, r_wheel, b_wheel);
                            finalAngle = motor_M2.getRotation() + getrevolutions(91, r_wheel, b_wheel);
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M2.getRotation());
                            dir = 'B';
                            storePath(dir, pathLength, path);
                            motion_state = MotionState::UTURN;
                        }
                    }
                    else
                    {
                        // Normal line-following

                        // Controller implementation
                        deadEndCounter = 0;
                        v = 0.3f * wheel_vel_max * r_wheel;
                        w = Kp * angle;
                        robot_coord = {v, w};
                        wheel_speed = Cwheel2robot.inverse() * robot_coord;

                        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
                        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));
                    }

                    break;
                }
                case MotionState::PREPARE_TURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M1.getRotation() - fixedAngle) < 0.01)
                    {
                        printf("Angle: %f, leftmean: %f, centerMean: %f, rightMean: %f\n", angle, leftMean, centerMean, rightMean);
                        // scout ahead. Checking if there is a straight line ahead of a left turn
                        if (std::max(leftMean, std::max(centerMean, rightMean)) == centerMean)
                        {
                            if (dir == 'L')
                            {
                                dir = 'S';
                            }
                        }

                        // if it still detects a long bar after advancing to turn, then it should stop.
                        if (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f) // end line detected
                        {
                            status = 1;
                            motion_state = MotionState::STOP;
                        }
                        else if (dir == 'R')
                        {
                            // RIGHT 90 DEGREE TURN
                            fixedAngle = motor_M1.getRotation() - 0.125; // other wheel pulls back slightly to adjust for turn error
                            finalAngle = motor_M2.getRotation() + getrevolutions(90, r_wheel, b_wheel);
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M2.getRotation());

                            storePath(dir, pathLength, path);
                            motion_state = MotionState::RIGHT_TURN;
                        }
                        else if (dir == 'L')
                        {
                            // LEFT 90 DEGREE TURN
                            finalAngle = motor_M1.getRotation() + getrevolutions(90, r_wheel, b_wheel);
                            fixedAngle = motor_M2.getRotation() - 0.125; // other wheel pulls back slightly to adjust for turn error
                            // printf("Final Angle: %.2f, curent angle: %.2f\n", finalAngle, motor_M1.getRotation());

                            storePath(dir, pathLength, path);
                            motion_state = MotionState::LEFT_TURN;
                        }
                        else
                        {
                            storePath(dir, pathLength, path);
                            motion_state = MotionState::FOLLOWLINE;
                        }
                    }
                    break;
                }
                case MotionState::LEFT_TURN:
                {
                    motor_M1.setRotation(finalAngle);
                    motor_M2.setRotation(fixedAngle);

                    // After the motor has completed the turn angle, revert to following the line
                    if (std::fabs(motor_M1.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::RIGHT_TURN:
                {

                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    // After the motor has completed the turn angle, revert to following the line
                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }
                    break;
                }
                case MotionState::UTURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    // After the motor has completed the turn angle, revert to following the line
                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::STOP:
                {
                    printf("stopping\n");
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);

                    robot_state = RobotState::SLEEP;
                    break;
                }
                default:
                    break;
                }
                break;
            }
            case RobotState::RUN:
            {
                printf("running.....\n");
                switch (motion_state)
                {
                case MotionState::FOLLOWLINE:
                {
                    // detecting intersections
                    leftTurn = (leftMean > 0.75f && centerMean > 0.5f && rightMean <= 0.5f);
                    rightTurn = (rightMean > 0.75f && centerMean > 0.5f && leftMean <= 0.5f);
                    crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
                    // isDeadEnd = (leftMean < 0.1f && centerMean < 0.1f && rightMean < 0.1f);
                    isIntersection = (leftTurn || rightTurn || crossRoad);

                    if (isIntersection)
                    {
                        dir = path[pathIndex]; // the turn direction is retrieved from the learned path array
                        pathIndex++;

                        // Robot needs to move forward 35-40mm to check if there is a straight line ahead of an intersection
                        // 0.125 is the number of rotation required of 35-40mm given wheel radius
                        // 0.125 = 40mm/(2*pi*r_wheel)
                        fixedAngle = motor_M1.getRotation() + 0.125;
                        finalAngle = motor_M2.getRotation() + 0.125;
                        motion_state = MotionState::PREPARE_TURN;
                    }
                    else
                    {
                        // Normal line-following

                        // Controller implementation
                        v = 0.3f * wheel_vel_max * r_wheel;
                        w = Kp * angle;
                        robot_coord = {v, w};
                        wheel_speed = Cwheel2robot.inverse() * robot_coord;

                        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
                        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));
                    }
                    break;
                }
                case MotionState::PREPARE_TURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M1.getRotation() - fixedAngle) < 0.01)
                    {
                        // if it still detects a long bar after advancing to turn, then it should stop.
                        if (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f) // end line detected
                        {
                            status = 1;
                            motion_state = MotionState::STOP;
                        }
                        else if (dir == 'R')
                        {
                            // RIGHT 90 DEGREE TURN
                            fixedAngle = motor_M1.getRotation() - 0.125; // fixed
                            finalAngle = motor_M2.getRotation() + getrevolutions(90, r_wheel, b_wheel);

                            motion_state = MotionState::RIGHT_TURN;
                        }
                        else if (dir == 'L')
                        {
                            // LEFT 90 DEGREE TURN
                            finalAngle = motor_M1.getRotation() + getrevolutions(90, r_wheel, b_wheel);
                            fixedAngle = motor_M2.getRotation() - 0.125; // fixed

                            motion_state = MotionState::LEFT_TURN;
                        }
                        else
                        {
                            motion_state = MotionState::FOLLOWLINE;
                        }
                    }
                    break;
                }
                case MotionState::LEFT_TURN:
                {
                    motor_M1.setRotation(finalAngle);
                    motor_M2.setRotation(fixedAngle);

                    if (std::fabs(motor_M1.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::RIGHT_TURN:
                {

                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }
                    break;
                }
                case MotionState::UTURN:
                {
                    motor_M1.setRotation(fixedAngle);
                    motor_M2.setRotation(finalAngle);

                    if (std::fabs(motor_M2.getRotation() - finalAngle) < 0.01)
                    {
                        motion_state = MotionState::FOLLOWLINE;
                    }

                    break;
                }
                case MotionState::STOP:
                {
                    printf("stopping\n");
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);
                    pathIndex = 0;
                    robot_state = RobotState::SLEEP;
                    break;
                }
                default:
                    break;
                }

                // robot_state = RobotState::SLEEP; // this is just for the testing.
                break;
            }
            }
        }
        else
        {
            if (do_reset_all_once)
            {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                robot_state = RobotState::INITIAL;
                // motion_state = MotionState::STOP;
                counter = 0;
            }
        }

        user_led = !user_led;
        counter++;

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

void storePath(char &direction, unsigned char &pathLength, char (&path)[100])
{
    path[pathLength] = direction; // Store the intersection in the path variable.
    pathLength++;
    for (int i = 0; i < pathLength; i++)
    {
        printf("%c", path[i]);
    }
    printf("\n");
    simplifyPath(path, pathLength); // Simplify the learned path.
}

void simplifyPath(char (&path)[100], unsigned char &pathLength)
{
    // The CREDIT for the simplifyPath() function is to
    // Patrick McCabe (https://patrickmccabemakes.com) for the path Solving Code

    // only simplify the path if the second-to-last turn was a 'B'
    if (pathLength < 3 || path[pathLength - 2] != 'B')
        return;

    int totalAngle = 0;
    int i;
    for (i = 1; i <= 3; i++)
    {
        switch (path[pathLength - i])
        {
        case 'R':
            totalAngle += 90;
            break;
        case 'L':
            totalAngle += 270;
            break;
        case 'B':
            totalAngle += 180;
            break;
        }
    }

    // Get the angle as a number between 0 and 360 degrees.
    totalAngle = totalAngle % 360;

    // Replace all of those turns with a single one.
    switch (totalAngle)
    {
    case 0:
        path[pathLength - 3] = 'S';
        break;
    case 90:
        path[pathLength - 3] = 'R';
        break;
    case 180:
        path[pathLength - 3] = 'B';
        break;
    case 270:
        path[pathLength - 3] = 'L';
        break;
    }

    // The path is now two steps shorter.
    pathLength -= 2;
}