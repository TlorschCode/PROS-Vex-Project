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

/// CONSTRUCTORS ///
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


/// VARIABLES, ARRAYS, LISTS, ECT ///
bool pressing_speed = {false};
bool pressing_stake = {false};
bool auton_y = {true};
bool auton_x = {true};
bool auton_rot = {true};
bool finish_move = {true};
bool within_x, within_y = {0};
int left_analog = {};
int up_analog = {};
int reversed = {1};
float PID_dist = {};
float p_gain = {6.0f};
float i_gain = {0.05f};
float d_gain = {4.2f};
float d_mod_x = {};
float d_mod_y = {};
float i_rot_gain = {0.05f};
float d_rot_gain = {0.85f};
float p_x = {};
float p_y = {};
float p_rot = {};
float i_x = {};
float i_y = {};
float i_rot = {};
float d_x = {};
float d_y = {};
float PID_x = {};
float PID_y = {};
float d_rot = {};
float left_speed, right_speed = {};
float max_speed = {85.0f};
float start_rot = {};
float rot = {};
float raw_rot = {};
float target_rot = {};
float original_rot = {};
float p_rot_gain = {0.2f};
float PID_rot = {};
float rot_diff = {};
float x_div = {};
float dist = {};
float original_x, original_y = {};
float x, y = {};        // X and Y of robot (h, k correspond to x, y in equation to find intercept)
float r = {6.0f};       // Circle's radius
float overall_velocity = {};
float rot_radians = {};
float x_diff, y_diff {};
float conveyor_speed = {440};
float lookahead = {2};
float pointX1, pointX2 = {};
float pointY1, pointY2 = {}; 
float minX = {};
float maxX = {};
float minY = {};
float maxY = {};
double discriminate = {};
double a, b, c = {};
double t1, t2 = {};
double x_intercept1, x_intercept2 = {};
double y_intercept1, y_intercept2 = {};
// points for auton
vector <float> points_x {0.0f, 0.0f, 12.0f, 24.0f};
vector <float> points_y {0.0f, 12.0f, 24.0f, 12.0f};
vector <bool> points_mode {0, 0, 0, 1}; // 0 is Pure Pursuit, 1 is PID
// constants
const double PI = 3.14159265358979323846;

// A = 1 + m**2
// B = 2 * (m * (y_intercept - y) - x)
// C = (x**2 + (y_intercept - y)**2 - r**2)
// discriminant = B**2 - 4*A*C   (from quadratic equation)

////| NON-DEFAULT FUNCTIONS |////
double min(double input1, double input2) {
	if (input1 < input2) {
		return input1;
	} else if (input2 < input2) {
		return input2;
	} else {
		return 0;
	}
}

double max(double input1, double input2) {
	if (input1 > input2) {
		return input1;
	} else if (input2 > input2) {
		return input2;
	} else {
		return 0;
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

void move_motors(float speedleft, float speedright) {
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

int sign(float input) {
	if (input >= 0) {
		return 1;
	} else {
		return -1;
	}
}

double map_value(float input, float input_start, float input_end, float output_start, double output_end) {
    return output_start + (output_end - output_start) * ((input - input_start) / (input_end - input_start));
}

void check_pause_program() { /// REMOVE THIS FUNCTION
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		brake_wheels();
		while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			wait(10);
		}
	}
}

void speed_control(float maxspeed = 50, bool auton = false, float minspeed = 5) {
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
	if (abs(left_speed) < 1) {
		left_speed *= 10;
	} else if (abs(right_speed) < 1) {
		right_speed *= 10;
	}
}

void check_reverse_motors() {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
		if (reversed == 1) {
			reversed = -1;
		} else {
			reversed = 1;
		}
	}
}

void control_motors(float up, float left) {
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed += 40;
		}
		pressing_speed = true;
	// DOWN //
	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		// CHECK ALREADY PRESSING //
		if (!pressing_speed) {
			max_speed -= 40;
		}
		pressing_speed = true;
	} else {
		// ALREADY PRESSING TOGGLE //
		pressing_speed = false;
	}
	if (max_speed < 5) {
		max_speed = 5;
	} else if (max_speed > 120) {
		max_speed = 120;
	}
	left_speed = (up * reversed) - left;
	right_speed = (up * reversed) + left;
	/// SPEED CONTROL ///
	speed_control(max_speed);
	move_motors(left_speed, right_speed);
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
	overall_velocity = (truncate(top_right.get_actual_velocity()) + truncate(bottom_right.get_actual_velocity()) + truncate(top_left.get_actual_velocity()) + truncate(bottom_left.get_actual_velocity())) / 4;
	y = y + (((((overall_velocity / 360) * 0.1f) * 12.56f) / 1.17f) * cos(rot_radians));
	x = x + (((((overall_velocity / 360) * 0.1f) * 12.56f) / 1.17f) * sin(rot_radians));
	//           -         -        -            Change this ^^^^^^ as needed BECAUSE VEX ODOMETRY IS STUPID
}

void auton_control(float targetx, float targety) {
	update_position();
	target_rot = to_degrees(atan2((targetx - x), (targety - y)));
	rot_diff = target_rot - rot;
	dist = sqrt(abs(pow(targetx - x, 2) + pow(targety - y, 2)));
	x_diff = targetx - x;
	y_diff = targety - y;
//? Don't question the logic, trust.
	auton_x = abs(x_diff) > 1.3f;
	auton_y = abs(y_diff) > 1.3f;
	if (abs(rot_diff) <= 3 && abs(x_diff) > 1.5f && abs(y_diff) > 1.5f) {
		if (auton_rot = true) {
			controller.rumble(".");
		}
		auton_rot = false;
	} else if ((abs(x_diff) > 1.5f && abs(y_diff) > 1.5f)) {
		auton_rot = true;
	}
}

//| This function cost me time, pain, tears, and lastly, my sanity. Enjoy.
void PID(float tarx, float tary) {
	float targx = tarx + 1;
	float targy = tary + 1;
	//|    PID    |//
	//| distance -
	p_x = (targx - x) * p_gain;
	p_y = (targy - y) * p_gain;
	i_x = (i_x + p_x) * sin(rot_radians);
	i_y = (i_y + p_y) * cos(rot_radians);
	if (abs(original_x) - x < 12) {
		d_mod_x = 1;
	} else {
		d_mod_x = 12 / abs(original_x - x);
	}
	if (abs(original_y) - x < 12) {
		d_mod_y = 1;
	} else {
		d_mod_y = 12 / abs(original_y - y);
	}
	d_x = (original_x - (x * d_gain)) * d_mod_x;
	d_y = (original_y - (y * d_gain)) * d_mod_y;
	PID_x = p_x + (d_x * d_gain) * sin(rot_radians); // Cos and Sin used down here because they
	PID_y = p_y + (d_y * d_gain) * cos(rot_radians); // were already used for i
	PID_dist = ((PID_y + (i_y * i_gain)) + (PID_x + (i_x * i_gain))) / 2;
	//| rotation - 
	p_rot = rot_diff * p_rot_gain;
	i_rot = i_rot + p_rot;
	d_rot = original_rot - rot;
	PID_rot = ((p_rot + (i_rot * i_rot_gain) + (d_rot * d_rot_gain)) * auton_rot);
}

void move_to(float tarx, float tary, float prevx = 0.0f, float prevy = 0.0f, bool pid = 1) {
	auton_control(tarx, tary);
	pointX1 = prevx;
	pointX2 = tarx;
	pointY1 = prevy;
	pointY2 = tary;
	clear_screen();
	auton_x = abs(x_diff) > 1;
	auton_y = abs(y_diff) > 1;
	auton_rot = abs(rot_diff) > 3;
	original_rot = rot;
	original_x = x;
	original_y = y;
	println(auton_x);
	wait(1000);
	// DONE: Get the robot to go to a target position and target rotation.
	// DONE: Turning PID
	// DONE: Movement PID
	// TODO: Add Pure Pursuit
	// TODO: Make Pure Pursuit work with PID
	// TODO: Make the robot be able to cycle between Pure Pursuit points and target points.
	if (pid) {
		while (auton_y || auton_x) {
			//|   PID   |//
			check_pause_program();
			PID(tarx, tary);
			left_speed = PID_dist - PID_rot;
			right_speed = PID_dist + PID_rot;
			println(x, 1);
			println(y, 2);
			println(rot, 3);
			println(auton_rot, 5);
			move_motors(left_speed, right_speed);
			auton_control(tarx, tary);
			wait(10);
		}
	} else {
		while (dist > lookahead) {
			//|   Pure Pursuit   |//
			check_pause_program();
			//| MaTHsuCKs
			// tarx = pointX2
			// prevx = pointX1
			// tary = pointY2
			// prevy = pointY1
			a = pow(tarx - prevx, 2) + pow(tary - prevy, 2);
			b = 2 * (((prevx - x) * (tarx - prevx)) + ((prevy - y) * (tary - prevy)));
			c = (pow(prevx - x, 2) + pow(prevy - y, 2)) - pow(r, 2);
			discriminate = pow(b, 2) - (4 * a * c);
			t1 = (-b + sqrt(discriminate)) / (2 * a);
			t2 = (-b - sqrt(discriminate)) / (2 * a);
			x_intercept1 = pointX1 + (pointX2 = pointX1) * t1;
			x_intercept1 = pointX1 + (pointX2 = pointX1) * t2;
			x_intercept1 = pointY1 + (pointY2 = pointY1) * t1;
			x_intercept1 = pointY1 + (pointY2 = pointY1) * t2;
			within_x = (minX <= x_intercept1 && x_intercept1 <= maxX) || (minX <= x_intercept2 && x_intercept2 <= maxX);
			within_y = (minY <= y_intercept1 && y_intercept1 <= maxY) || (minY <= y_intercept2 && y_intercept2 <= maxY);
			//| lahjick :/
			if (discriminate >= 0) {
				if (within_x && within_y) {
					if (abs(x_intercept2 - tarx) + abs(y_intercept2 - tary) < abs(x_intercept1 - tarx) + abs(y_intercept1 - tary)) {
						PID(x_intercept2, y_intercept2);
						println(x_intercept2, 1);
						println(y_intercept2, 2);
						println(rot, 3);
						println("Intercept 2", 4);
					} else {
						PID(x_intercept1, y_intercept1);
						println(x_intercept1, 1);
						println(y_intercept1, 2);
						println(p_x);
					}
				}
			}
			// discriminant = pow(b, 2) - 4*A*C   (from quadratic equation);
			left_speed = PID_dist - PID_rot;
			right_speed = PID_dist + PID_rot;
			println(x_intercept1);
			println(x_intercept2, 2);
			println(y_intercept1, 3);
			println(y_intercept2, 4);
			println(PID_dist, 5);
			move_motors(left_speed, right_speed);
			auton_control(tarx, tary);
			wait(10);
		}
	}
	println("DONE", 4);
	controller.rumble("-.");
	wait(100);
	brake_wheels();
	println("DONE WITH AUTON", 1);
	wait(1 * 1000);
	clear_screen();
}

////| DEFAULT FUNCTIONS |////
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
	// Hardware Config
	top_right.set_reversed(true);
	bottom_right.set_reversed(true);
	top_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bottom_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	top_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bottom_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Resetting inertial sensor
	inert.reset();
	wait(2 * 1000);
	inert.tare();
	wait(100);
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
	/// SETUP ///
	println(get_rot());
	wait(100);
	move_to(12, 12, 0, 0, 0);
	// for (size_t i = 0; i < points_x.size(); i++) {
	// 	move_to(points_x[i], points_y[i], points_x[i + 1], points_y[i + 1], points_mode[i]);
	// 	controller.rumble(".");
	// 	wait(1000);
	// }
	
	/// a = 1 + pow(m, 2);
	/// b = 2 * (m * (y_intercept - k) - h);
	/// c = (pow(h, 2) + pow((y_intercept - k), 2) - pow(r, 2));

	
	update_position();
		
	// brake_wheels();
	// brake_wheels();
}

void opcontrol() {
	autonomous(); //Autonomous ***REMOVE*** FOR COMP//

	/// INIT ///
	println("OPCONTROL");
	brake_wheels();
	wait(1000);
	clear_screen();
	wait(10);

	/// CODE ///
	while (true)
	{
		up_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		left_analog = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 2.3f;
		check_reverse_motors();
		control_motors(up_analog, left_analog);
		speed_control(max_speed);
		control_scoring();
		wait(10);
	}
}
