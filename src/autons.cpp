#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

pros::adi::DigitalOut AutonClimb('A');
pros::adi::DigitalOut AutonIntakeLift('B');
pros::adi::DigitalOut AutonHopperLift('C');
pros::adi::DigitalOut AutonMoGoClamp('D');
pros::adi::DigitalOut AutonFlag('E');

pros::Motor AutonIntake1 (14, pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);
pros::Motor AutonIntake2 (16, pros::v5::MotorGear::green, pros::v5::MotorUnits::counts);
pros::Motor AutonHopper (10, pros::v5::MotorGear::red, pros::v5::MotorUnits::counts);

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(7.69, 0.25, 11.5);
  chassis.pid_turn_constants_set(4, 0.2, 32, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-48_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(300_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(330_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(345_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(360_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(2000);

}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .
void right_side_b() {
  AutonIntake1.move(-127);
  AutonMoGoClamp.set_value(true);
  pros::delay(100);
  AutonIntake1.move(127);
  chassis.pid_drive_set(16_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90, TURN_SPEED);
  chassis.pid_wait();
  AutonIntakeLift.set_value(true);
  chassis.pid_drive_set(6_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 20);
  chassis.pid_wait();
  AutonIntakeLift.set_value(false);
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonIntake1.move(0);
  chassis.pid_turn_relative_set(60, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-42_in, 50, true);
  pros::delay(1100);
  AutonMoGoClamp.set_value(false);
  AutonIntake2.move(127);
  pros::delay(500);
  AutonIntake1.move(127);
  pros::delay(2000);
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();
  AutonIntake1.move(0);
  chassis.pid_drive_set(50_in, 127);
  chassis.pid_wait();
}

void left_side_b() {
  AutonIntake1.move(-127);
  pros::delay(100);
  AutonIntake1.move(0);
  AutonMoGoClamp.set_value(true);
  chassis.pid_drive_set(-46_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  AutonMoGoClamp.set_value(false);
  chassis.pid_turn_relative_set(180, 70);
  default_constants();
  chassis.pid_drive_set(10_in, 50);
}

void right_side_r() {
  chassis.pid_drive_set(50_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonFlag.set_value(true);
  pros::delay(250);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonFlag.set_value(false);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90, TURN_SPEED);
  AutonHopper.move_absolute(750, 100);
  AutonIntakeLift.set_value(true);
  chassis.pid_wait();
  AutonIntake1.move(127);
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonIntakeLift.set_value(false);
  pros::delay(100);
  chassis.pid_turn_relative_set(-45, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-36_in, 60);
  pros::delay(1500);
  AutonMoGoClamp.set_value(true);
  AutonHopper.move_absolute(300, 100);
  pros::delay(1000);
  chassis.pid_wait();
  AutonIntake2.move(127);
  pros::delay(1000);
  AutonIntake2.move(0);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(0,0);
  pros::delay(1000);
  AutonIntake2.move(127);
}

void awp_r(){
  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-30, TURN_SPEED);
  AutonHopper.move_absolute(1750, 100);
  chassis.pid_wait();
  AutonHopperLift.set_value(true);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonHopper.move_absolute(300, 100);
  pros::delay(750);
  chassis.pid_drive_set(-50_in, 90);
  chassis.pid_wait();
  AutonMoGoClamp.set_value(true);
  AutonHopperLift.set_value(false);
  pros::delay(250);
  chassis.pid_turn_relative_set(90, TURN_SPEED);
  chassis.pid_wait();
  AutonIntake1.move(127);
  AutonIntake2.move(127);
  chassis.pid_drive_set(14_in, 127);
  chassis.pid_wait();
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_drive_set(-50_in, DRIVE_SPEED);
  chassis.pid_wait();
  AutonIntake1.move(0);
  AutonIntake2.move(0);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(-127, -127);
  pros::delay(100);
  chassis.drive_set(127, 127);
  pros::delay(100);
  chassis.drive_set(0,0);
  AutonIntake2.move(127);
  pros::delay(1000);
  AutonMoGoClamp.set_value(false);
  AutonIntake1.move(0);
  AutonIntake2.move(0);
  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  AutonHopper.move_absolute(1750,100);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45, TURN_SPEED);
  chassis.pid_drive_set(46_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void prog_skills() {
  AutonMoGoClamp.set_value(true);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED);
  pros::delay(700);
  AutonMoGoClamp.set_value(false);
  AutonIntake2.move(127);
  chassis.pid_turn_relative_set(90, TURN_SPEED);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  AutonMoGoClamp.set_value(true);

}