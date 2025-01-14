#include "main.h"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include <iostream>
#include "vector"
#include <cmath>
#include <string>
#include <type_traits>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this is the code currently being used on our robot. I messed up the motion tracking a little bit, so I wanated to keep my previous commit now titled motion_tracking_commit.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;

//// CONSTRUCTORS ////
// pneumatics -
pros::adi::Pneumatics stake_lift2(1, true);
pros::adi::Pneumatics stake_lift1(1, false);
pros::adi::Pneumatics stake_clamp(1, true);

// sensors -
pros::IMU inert(5);

// motors -
pros::Motor top_left(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_left(2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor top_right(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_right(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intake(6, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);
pros::Motor conveyor(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// controller -
pros::Controller controller(pros::E_CONTROLLER_MASTER);


//// VARIABLES, ARRAYS, LISTS, ECT ////
bool pressing_speed = {false};
bool pressing_stake = {false};
int left_analog = {};
int up_analog = {};
float left_speed, right_speed = {};
float max_speed = {85.0f};
float start_rot = {};
float rot = {};
float target_rot = {};
float xdiv = {};
float dist = {};
float x, y = {};        // X and Y of robot
float h, k = {};        // Center of circle
float r = {5.0f};       // Circle's radiusy
float m = {1.0f};       // Slope of line between two points
float a, b, c = {};     // A, B, and C for quadratic equation
float y_intercept = {1.0f};
float overall_velocity = {};
float overall_y = {};
float overall_x = {};
float rot_radians = {};
float x_diff, y_diff {};
// points for auton
vector <float> auton_x {12.0f, 12.0f};
vector <float> auton_y {12.0f, 24.0f};
// constants
const double pi = 3.14159265358979323846;

// A = 1 + m**2
// B = 2 * (m * (y_intercept - k) - h)
// C = (h**2 + (y_intercept - k)**2 - r**2)
// discriminant = B**2 - 4*A*C

//// NON-DEFAULT FUNCTIONS ////
void grab_stake() {
	stake_lift1.toggle();
	stake_lift2.toggle();
	stake_clamp.toggle();
}

void move_wheels(float speedleft, float speedright) {
	top_left.move_velocity(speedleft);
	bottom_left.move_velocity(speedleft);
	top_right.move_velocity(speedright);
	bottom_right.move_velocity(speedright);
}

void brake_wheels() {
	top_left.brake();
	bottom_left.brake();
	top_right.brake();
	bottom_right.brake();
}

bool clear_screen() {
	pros::lcd::clear_line(6);
	pros::lcd::clear_line(5);
	pros::lcd::clear_line(4);
	pros::lcd::clear_line(3);
	pros::lcd::clear_line(2);
	pros::lcd::clear_line(1);
	return true;
}

bool wait(float time) {
	pros::delay(time);
		return true;
}

template <typename T>
void println(const T& input, int row = 1) {
    std::string printtext;
    if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, const char*>) {
        // Handle string and C-style string types
        printtext = input;
    } else {
        // Handle other types using stringstream
        std::stringstream ss;
        ss << input;
        printtext = ss.str();
    }
    pros::lcd::set_text(row, printtext);
}

float get_rot() {
	// println(floor(inert.get_rotation() * 100) / 100);
	return floor(inert.get_rotation() * 100) / 100;
}

void speed_control(float maxspeed) {
	if (left_speed > maxspeed) {
		left_speed = maxspeed;
	} else if (left_speed < (maxspeed * -1)) {
		left_speed = maxspeed * -1;
	}
	if (right_speed > max_speed) {
		right_speed = max_speed;
	} else if (right_speed < (max_speed * -1)) {
		right_speed = max_speed * -1;
	}
}

void control_motors(float up, float left) {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) 
	{
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed += 10;
			if (max_speed > 200) {
				max_speed = 200;
			}
		}
		pressing_speed = true;
	// DOWN //
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) 
	{
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed -= 10;
			if (max_speed < 10) {
				max_speed = 10;
			}
		}
		pressing_speed = true;
	} else 
	{
		// ALREADY PRESSING TOGGLE //
		pressing_speed = false;
	}
	float left_speed = left - up;
	float right_speed = left + up;
	//// SPEED CONTROL ////
	speed_control(max_speed);
	left_speed *= -1;
	right_speed *= -1;
	left_speed = left_speed * 1 + (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 2);
	right_speed = right_speed * 1 + (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 2);
	move_wheels(left_speed, right_speed);
}

void control_scoring() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		if (pressing_stake = false) {
			grab_stake();
		}
		pressing_stake = true;
	} else {
		pressing_stake = false;
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move_velocity(100);
		conveyor.move_velocity(75);
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		intake.move_velocity(-100);
		conveyor.move_velocity(-75);
	}
}

void update_position() {  // Tracks and changes the robot's position based on odometry information
	rot = get_rot();
	rot_radians = fmod(rot, 360) * (pi / 180);
	overall_velocity = ((top_right.get_actual_velocity() + bottom_right.get_actual_velocity()) - top_left.get_actual_velocity() + bottom_left.get_actual_velocity()) / 4;
	overall_y = overall_y + (((((overall_velocity / 360) * 0.1f) * 11.65f) * cos(rot_radians)) * 2);  // CHANGE 11.65 AS NEEDED ***DEBUG***
	overall_x = overall_x + (((((overall_velocity / 360) * 0.1f) * 13.50f) * sin(rot_radians)) * 2);  // CHANGE 12.56 AS NEEDED ***DEBUG***
}

void move_to(float tarx, float tary) {
	x_diff = tarx - overall_x;
	y_diff = tary - overall_y;
	x = tarx;
	y = tary;
	target_rot = atan2((x - overall_x), (y - overall_y));
	start_rot = target_rot - rot;
	dist = sqrt(abs(pow(x - overall_x, 2) + pow(y - overall_y, 2)));
	
	// right_speed *= -1;
	// left_speed *= 3;
	// right_speed *= 3;

	while (/*tary - overall_y > 0.1 && */tarx - overall_x > 0.1) {
		wait(10);
		target_rot = atan2((x - overall_x), (y - overall_y));
		xdiv = start_rot - (target_rot - rot);
		left_speed = dist + xdiv;
		right_speed = dist - xdiv;
		left_speed *= -1;
		// right_speed *= -1;
		left_speed *= 2;
		right_speed *= 2;
		speed_control(30);
		move_wheels(left_speed, right_speed);
		println(left_speed);
		println(right_speed, 2);
		update_position();
	}
	brake_wheels();
}

//// DEFAULT FUNCTIONS ////
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Start
	pros::lcd::initialize();
	println("Starting: ALL");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	println("Start Already!!");
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
	//// SETUP ////
	inert.tare();
	inert.reset();
	while (inert.is_calibrating()) {
		1;
	}
	move_to(24, 24);
	
	// 		// a = 1 + pow(m, 2);
	// 		// b = 2 * (m * (y_intercept - k) - h);
	// 		// c = (pow(h, 2) + pow((y_intercept - k), 2) - pow(r, 2));

	
	update_position();
		
	brake_wheels();
}


void opcontrol() {
	
	// auton
	// autonomous(); //Autonomous ***REMOVE*** FOR COMP//

	//// INIT ////
	pros::lcd::set_text(1, "OPCONTROL");
	wait(1000);
	
	//// CODE ////
	while (true)
	{
		left_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2;
		up_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		control_motors(up_analog, left_analog);
		control_scoring();

		wait(10);
	}
}
