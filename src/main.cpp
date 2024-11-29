#include "main.h"

#include "EZ-Template/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"

// Ports
// 1, 2, 3, 8, 10, 11, 13, 15, 18 all bad

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

pros::adi::DigitalOut climbClamp('A');
pros::adi::DigitalOut intakeLift('B');
pros::adi::DigitalOut hopperPiston('C');
pros::adi::DigitalOut moGoClamp('D');
pros::adi::DigitalOut flagPiston('E');
pros::Motor Intake1 (14, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
pros::Motor Intake2 (15, pros::v5::MotorGear::green, pros::v5::MotorUnits::counts);
pros::Motor Hopper (16, pros::v5::MotorGear::green, pros::v5::MotorUnits::counts);
pros::Controller master (pros::E_CONTROLLER_MASTER);


// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-11, -12, -13},  // Left Chassis Ports (negative port will reverse it!)
    {18, 19, 20},    // Right Chassis Ports (negative port will reverse it!)

    1,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


typedef struct {
    int pos;
} motor_args;

void setHopper(int speed) {
    Hopper.move(speed);
}

void liftTask() {
    while (true){
        Hopper.brake();
        pros::delay(20);
    }
}

pros::Task LiftTask(liftTask);

//ez::PID hopperPID{50, 0, 0, 0, "Hopper"};

/*void lift_wait() {
    while (hopperPID.exit_condition(Hopper, true) == ez::RUNNING) {
        pros::delay(ez::util::DELAY_TIME);
    }
}*/

/*void lift_task() {
    pros::delay(2000);
    while (true) {
        setHopper(hopperPID.compute(Hopper.get_position()));
        pros::delay(ez::util::DELAY_TIME);
    }
}*/
//pros::Task Lift_Task(lift_task);

// motor_args* ma = new motor_args();

// bool hopperOn = false;

// void resetMotor(void* params){
//     hopperOn = true;
//     int hpos = ((motor_args*)params)->pos;
//     while(Hopper.get_position() > hpos + 5 || Hopper.get_position() < hpos - 5) {
//         hpos = ((motor_args*)params)->pos;
//         Hopper.move_absolute(hpos, 100);
//         pros::delay(20);
//     }
//     hopperOn = false;
// }

// void runHopper(void* params) {
//     int pos = ((motor_args*)params)->pos;
//     while(Hopper.get_position() > pos + 5 || Hopper.get_position() < pos - 5) {
//         Hopper.move_absolute(pos, 100);
//         pros::delay(20);
//     }
    
// }


void initialize() {
    pros::delay(500);  // Stop the user from doing anything while legacy ports configure

    // Configure your chassis controls
    chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
    chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
    chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

    // Set the drive to your own constants from autons.cpp!
    default_constants();

    // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
    // chassis.opcontrol_ curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
    // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

    // Autonomous Selector using LLEMU
    ez::as::auton_selector.autons_add({
        //Auton("PID auton", drive_example), 
        
        //Auton("Blue side AWP", awp_b),
        Auton("Red side AWP", awp_r),
        
        Auton("Left Side Blue auton", left_side_b),
        Auton("Right Side Red auton", right_side_r),
        Auton("Programming Skills", prog_skills),

        
        Auton("Example Turn\n\nTurn 3 times.", drive_example),
        Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
        Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
        Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
        Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
        Auton("Combine all 3 movements", combining_movements),
        Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
    });

    // Initialize chassis and auton selector
    chassis.initialize();
    ez::as::initialize();
    Hopper.tare_position();
    //hopperPID.exit_condition_set(80, 50, 300, 150, 500, 500);
    Hopper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Hopper.brake();
    master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
    chassis.pid_targets_reset();                // Resets PID targets to 0
    chassis.drive_imu_reset();                  // Reset gyro position to 0
    chassis.drive_sensor_reset();               // Reset drive sensors to 0
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

    ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
    master.rumble(".");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    // This is preference to what you like to drive on
    pros::motor_brake_mode_e_t driver_preference_brake = pros::E_MOTOR_BRAKE_HOLD;

    chassis.drive_brake_set(driver_preference_brake);

    bool clampOn = false;
    bool climb = false;
    bool intake = false;
    bool flagDown = false;
    bool hopperDown = false;
    bool hopperMoved = false;
    int lIntake = 0;
    int uIntake = 0;
    int hopperCurPos = 50;


    // pros::task_t task = pros::c::task_create(resetMotor, (void*)ma, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Task Name");
    // pros::Task cpptask (task);

    while (true) {
        // PID Tuner
        // After you find values that you're happy with, you'll have to set them in auton.cpp
        // if (!pros::competition::is_connected()) {
        // // Enable / Disable PID Tuner
        // //  When enabled:
        // //  * use A and Y to increment / decrement the constants
        // //  * use the arrow keys to navigate the constants
        //     if (master.get_digital_new_press(DIGITAL_X))
        //         chassis.pid_tuner_toggle();

        // // Trigger the selected autonomous routine
        //     if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        //         autonomous();
        //         chassis.drive_brake_set(driver_preference_brake);
        //     }

        //     chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
        // }

        // chassis.opcontrol_tank();  // Tank control
        chassis.opcontrol_arcade_standard(ez::SPLIT);  // Standard split arcade
        // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
        // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
        // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

        // . . .
        // Put more user control code here!
        // . . .

        if (master.get_digital_new_press(DIGITAL_A)) { 
            clampOn = !clampOn;
            moGoClamp.set_value(clampOn);
        }

        if (master.get_digital_new_press(DIGITAL_B)) {
            climb = !climb;
            climbClamp.set_value(climb);
            if(!flagDown){
                flagDown = !flagDown;
                flagPiston.set_value(flagDown);
            }
        }

        if (master.get_digital_new_press(DIGITAL_RIGHT)) {
            hopperDown = !hopperDown;
            hopperPiston.set_value(hopperDown);
        }

        if (master.get_digital_new_press(DIGITAL_Y)) {
            flagDown = !flagDown;
            flagPiston.set_value(flagDown);
        }

        if (master.get_digital_new_press(DIGITAL_X)) {
            intake = !intake;

            if(hopperCurPos < 750 && intake) {
                /*hopperMoved = true;
                hopperPID.target_set(750);
                hopperCurPos = 750;*/
                Hopper.move_absolute(750,127);
            }

            intakeLift.set_value(intake);

            if(hopperMoved && !intake) {
                /*hopperMoved = false;
                hopperPID.target_set(0);
                hopperCurPos = 0;*/
                Hopper.move_absolute(0,127);
            }
        }

        if(master.get_digital(DIGITAL_R1)){
            Intake1.move(127);
        }
        else if(master.get_digital(DIGITAL_R2)){
            Intake1.move(-127);
        }
        else {
            Intake1.move(0);
        }

        if(master.get_digital(DIGITAL_L1)){
            Intake2.move(127);
        }
        else if(master.get_digital(DIGITAL_L2)){
            Intake2.move(-127);
        }
        else {
            Intake2.move(0);
        }


        // if (master.get_digital_new_press(DIGITAL_L1)) {
        //     if(uIntake == 1) {
        //         uIntake = 0;
        //         Intake2.move(0);
        //     } else if (uIntake != 1) {
        //         uIntake = 1;
        //         Intake2.move(127);
        //     }
        // }
        
        // if(master.get_digital_new_press(DIGITAL_L2)){
        //     if(uIntake == -1) {
        //         uIntake = 0;
        //         Intake2.move(0);
        //     } else if (uIntake != -1) {
        //         uIntake = -1;
        //         Intake2.move(-127);
        //     }
        // }

        if(master.get_digital(DIGITAL_UP)){
            //hopperCurPos += 5;
            //hopperPID.target_set(hopperCurPos);
            Hopper.move(127);
        }
        else if(master.get_digital(DIGITAL_DOWN)){
            //hopperCurPos -= 5;
            //hopperPID.target_set(hopperCurPos);
            Hopper.move(-127);
        }
        else {
            Hopper.move(0);
            Hopper.brake();
        }

        pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    }
}