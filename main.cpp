#include "main.h"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include <iostream>
#include "vector"
#include <cmath>

using namespace std;

//// CONSTRUCTORS ////
// sensors -
pros::IMU inert(5);

// motors -
pros::Motor top_left(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_left(2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor top_right(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_right(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// controller -
pros::Controller controller(pros::E_CONTROLLER_MASTER);


//// VARIABLES, ARRAYS, LISTS, ECT ////
bool pressing_speed = {false};
int left_analog = {};
int up_analog = {};
float left_speed, right_speed = {};
float max_speed = {25.0f};
float rot = {};
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
// points for auton
vector <float> auton_x {12.0f, 12.0f};
vector <float> auton_y {12.0f, 24.0f};
// constants
const double pi = 3.14159265358979323846;

// A = 1 + m**2
// B = 2 * (m * (y_intercept - k) - h)
// C = (h**2 + (y_intercept - k)**2 - r**2)
// discriminant = B**2 - 4*A*C
// The code above was written in python. It will find the intersect points of a line and a circle

//// NON-DEFAULT FUNCTIONS ////
bool clear_screen() {
	pros::lcd::clear_line(6);
	pros::lcd::clear_line(5);
	pros::lcd::clear_line(4);
	pros::lcd::clear_line(3);
	pros::lcd::clear_line(2);
	pros::lcd::clear_line(1);
	return true;
} // this function will clear the entire screen of the robot

bool wait(float time) {
	pros::delay(time);
	return true;
} // this function is a shortcut to wait a certain amount of time (milliseconds)

template <typename T>
void println(const T& input, int row = 1) {
	string printtext = to_string(input);
	pros::lcd::set_text(row, printtext);
} // this function is a shortcut to print to the robot's brain

float get_rot() {
	return floor(inert.get_rotation() * 100) / 100;
}

void control_motors(float up, float left) {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) 
	{
		if (!pressing_speed) {
			max_speed += 5;
			if (max_speed > 200) {
				max_speed = 200;
			}
		}
		pressing_speed = true;
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) 
	{
		if (!pressing_speed) {
			max_speed -= 5;
			if (max_speed < 5) {
				max_speed = 5;
			}
		}
		pressing_speed = true;
	} else 
	{
		pressing_speed = false;
	}
	float left_speed = left - up;
	float right_speed = left + up;
	//// SPEED CONTROL ////
	if (left_speed > max_speed) {
		left_speed = max_speed;
	} else if (left_speed < (max_speed * -1)) {
		left_speed = max_speed * -1;
	}
	if (right_speed > max_speed) {
		right_speed = max_speed;
	} else if (right_speed < (max_speed * -1)) {
		right_speed = max_speed * -1;
	}
	left_speed *= -1;
	right_speed *= -1;
	// move motors
	top_left.move_velocity(left_speed);
	bottom_left.move_velocity(left_speed);
	top_right.move_velocity(right_speed);
	bottom_right.move_velocity(right_speed);
}

//// DEFAULT FUNCTIONS ////
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "STARTING PROGRAM");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
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

void autonomous() {
	// for (int i; i < auton_x.size(); i++) {
	// 	float target_x = auton_x.at(i);
	// 	float target_y = auton_y.at(i);
	// 	float dist = sqrt(abs((target_x - x) + (target_y - y)));
	// 	dist = 10;
	// 	while (dist > 3) {
	// 		// a = 1 + pow(m, 2);
	// 		// b = 2 * (m * (y_intercept - k) - h);
	// 		// c = (pow(h, 2) + pow((y_intercept - k), 2) - pow(r, 2));

	// 		wait(10);
	// 	}
	// }
	
	// v position tracking v //
	rot = get_rot();
	rot_radians = fmod(rot, 360) * (pi / 180);
	overall_velocity = (top_left.get_actual_velocity() + bottom_left.get_actual_velocity() - (top_right.get_actual_velocity() + bottom_right.get_actual_velocity())) / 4;
	overall_y = overall_y + ((((overall_velocity / 360) * 0.1f) * 11.65f) * cos(rot_radians));  // CHANGE 11.65 AS NEEDED ***DEBUG***
	overall_x = overall_x + ((((overall_velocity / 360) * 0.1f) * 11.65f) * sin(rot_radians));  // CHANGE 11.65 AS NEEDED ***DEBUG***
	// the script above ^^^ will accurately (mostly) track the robot's position using the wheel positions and an imu for rotation
}


void opcontrol() {
	//// SETUP ////
	inert.tare();
	inert.reset();
	while (inert.is_calibrating()) {
		int a = 0;
	}
	// auton
	autonomous(); //Autonomous ***REMOVE*** FOR COMP//

	//// INIT ////
	pros::lcd::set_text(1, "OPCONTROL");
	wait(1000);
	
	//// CODE ////
	while (true)
	{
		left_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2;
		up_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		control_motors(up_analog, left_analog);

		wait(10);
	}
}
