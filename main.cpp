#include "main.h"
#include <iostream>
#include "vector"
#include <string>
#include <type_traits>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// this is the code currently being used on our robot. I messed up the motion tracking a little bit, so I wanated to keep my previous commit now titled motion_tracking_commit.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace std;

//// CONSTRUCTORS ////
// pneumatics -
pros::adi::Pneumatics stake_lift(1, true);

// sensors -
pros::IMU inert(5);

// motors -
pros::Motor top_left(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_left(2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor top_right(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor bottom_right(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intake(6, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);
pros::Motor conveyor(7, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

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
float raw_rot = {};
float target_rot = {};
float rot_diff = {};
float xdiv = {};
float dist = {};
float x, y = {};        // X and Y of robot
float h, k = {};        // Center of circle
float r = {5.0f};       // Circle's radiusy
float m = {1.0f};       // Slope of line between two points
float a, b, c = {};     // A, B, and C for quadratic equation
float y_intercept = {1.0f};
float overall_velocity = {};
float rot_radians = {};
float x_diff, y_diff {};
float conveyor_speed = {440};
// points for auton
vector <float> auton_x {12.0f, 12.0f};
vector <float> auton_y {12.0f, 24.0f};
// constants
const double PI = 3.14159265358979323846;

// A = 1 + m**2
// B = 2 * (m * (y_intercept - k) - h)
// C = (h**2 + (y_intercept - k)**2 - r**2)
// discriminant = B**2 - 4*A*C

//// * * * NON-DEFAULT FUNCTIONS * * * ////
void check_quit_program() { //// REMOVE THIS FUNCTION
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		exit(0);
	}
}

double to_radians(float degrees) {
	return degrees * (PI / 180);
}

float to_degrees(double radians) {
	return radians * (180 / PI);
}

double truncate(double num, int cutoff = 2) {
	return floor(num * pow(10, cutoff)) / pow(10, cutoff);
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

void clear_screen() {
	pros::lcd::clear_line(1);
	pros::lcd::clear_line(2);
	pros::lcd::clear_line(3);
	pros::lcd::clear_line(4);
	pros::lcd::clear_line(5);
	pros::lcd::clear_line(6);
}

void wait(float time) {
	pros::delay(time);
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

void speed_control(float maxspeed = 50, float minspeed = 10) {
	// Max Speed Control
	if (abs(left_speed) > maxspeed) {
		if (left_speed >= 0) {
			left_speed = maxspeed;
		} else if (left_speed <= 0) {
			left_speed = -1 * maxspeed;
		}
	}
	if (abs(right_speed) > maxspeed) {
		if (right_speed >= 0) {
			right_speed = maxspeed;
		} else if (right_speed <= 0) {
			right_speed = -1 * maxspeed;
		}
	}
	// Min Speed Control
	if (!minspeed == 0) {
		if (abs(left_speed) < minspeed) {
			if (left_speed >= 0) {
				left_speed = minspeed;
			} else if (left_speed <= 0) {
				left_speed = -1 * minspeed;
			}
		}
	}
}

void control_motors(float up, float left) {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed += 10;
			if (max_speed > 200) {
				max_speed = 200;
			}
		}
		pressing_speed = true;
	// DOWN //
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed -= 10;
			if (max_speed < 10) {
				max_speed = 10;
			}
		}
		pressing_speed = true;
	} else {
		// ALREADY PRESSING TOGGLE //
		pressing_speed = false;
	}
	float left_speed = left - up;
	float right_speed = left + up;
	//// SPEED CONTROL ////
	speed_control(max_speed);
	left_speed *= -1;
	right_speed *= -1;
	move_wheels(left_speed, right_speed);
}

void control_scoring() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		if (!pressing_stake) {
			stake_lift.toggle();
		}
		pressing_stake = true;
	} else {
		pressing_stake = false;
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move_velocity(conveyor_speed * -1);
		conveyor.move_velocity(conveyor_speed);
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		intake.move_velocity(conveyor_speed);
		conveyor.move_velocity(conveyor_speed * -1);
	} else {
		intake.brake();
		conveyor.brake();
	}
}

float get_rot() {
    raw_rot = inert.get_rotation();
    raw_rot = fmod(raw_rot + 180, 360);  // Shift range to [0, 360)
    if (raw_rot < 0) {
        raw_rot += 360;  // Handle negative results from fmod
    }
	raw_rot -= 180;
    return truncate(raw_rot);
}

void update_position() {  // Tracks and changes the robot's position based on odometry information
	rot = get_rot();
	rot_radians = to_radians(rot);
	overall_velocity = (truncate(top_right.get_actual_velocity()) + truncate(bottom_right.get_actual_velocity())) - (truncate(top_left.get_actual_velocity()) + truncate(bottom_left.get_actual_velocity())) / 4;
	y = y + (((((overall_velocity / 360) * 0.1f) * 11.65f) * cos(rot_radians)) * 2);  // CHANGE 11.65 AS NEEDED **DEBUG**
	x = x + (((((overall_velocity / 360) * 0.1f) * 13.50f) * sin(rot_radians)) * 2);  // CHANGE 12.56 AS NEEDED **DEBUG**
}

void auton_control(float targetx, float targety) {
	target_rot = to_degrees(atan2((targetx - x), (targety - y)));
	rot_diff = target_rot - rot;
	dist = sqrt(abs(pow(targetx - x, 2) + pow(targety - y, 2)));
}

void move_to(float tarx, float tary) {
	rot = get_rot();
	x_diff = tarx - x;
	y_diff = tary - y;
	auton_control(tarx, tary);
	clear_screen();
	println(target_rot);
	println(rot, 2);
	println(rot_diff, 3);
	println(rot_diff < 0.1f, 4);
	wait(3 * 1000);
	clear_screen();

	while (tary - y > 3 || tarx - x > 3 || abs(rot_diff) > 2) {
		check_quit_program();
		auton_control(tarx, tary);
		left_speed = rot_diff / -2;
		right_speed = rot_diff / -2;
		speed_control(10, 95);
		left_speed += y_diff / (rot_diff / 55);
		right_speed -= y_diff / (rot_diff / 55);
		speed_control(75, 15);
		println(y_diff);
		move_wheels(left_speed, right_speed);
		update_position();
		rot_diff = target_rot - rot;
		x_diff = tarx - x;
		y_diff = tary - y;
		wait(10);
	}
	brake_wheels();
	clear_screen();
	println("DONE WITH AUTON");
	wait(5 * 1000);
}

//// * * * DEFAULT FUNCTIONS * * * ////
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Start
	pros::lcd::initialize();
	clear_screen();
	println("Starting: ALL");
	get_rot();
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
	wait(2.5 * 1000);
	println(get_rot());
	wait(2.5 * 1000);
	move_to(12, 0);
	
	//// a = 1 + pow(m, 2);
	//// b = 2 * (m * (y_intercept - k) - h);
	//// c = (pow(h, 2) + pow((y_intercept - k), 2) - pow(r, 2));

	
	update_position();
		
	// brake_wheels();
	// brake_wheels();
}


void opcontrol() {
	
	// auton
	// autonomous(); //Autonomous ***REMOVE*** FOR COMP//

	//// INIT ////
	println("OPCONTROL");
	wait(1000);
	
	//// CODE ////
	while (true)
	{
		up_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		left_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 3;
		control_motors(up_analog, left_analog);
		speed_control(max_speed, 15);
		control_scoring();

		wait(10);
	}
}
