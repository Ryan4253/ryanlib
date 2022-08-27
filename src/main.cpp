#include "main.h"
#include "ryanlib/api.hpp"
#include "okapi/api.hpp"
using namespace okapi;

// Chassis' max velocity, acceleration, and jerk
ProfileConstraint moveLimit({5.3_ftps, 6_ftps2, 27_ftps3});

// TUNING, uncomment if you want to use the custom velocity controller
//FFVelocityController leftController(0.23, .025, .01, 1.8, .1);
//FFVelocityController rightController(0.23, .025, .01, 1.8, .1);

std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
	// motor port, negative if reversed
	.withMotors({14, 15, 16}, {17, 18, 20}) 
	// {motor cartridge, wheel gear / motor gear}, {{wheel diameter, track diameter} imev5<cart>TPR}
	.withDimensions({AbstractMotor::gearset::blue, 48.0/36.0}, {{2.75_in, 1.25_ft}, imev5BlueTPR}) 
	.build();

std::shared_ptr<AsyncMotionProfiler> profiler = AsyncMotionProfilerBuilder()
	.withOutput(chassis)
	.withProfiler(std::make_unique<SCurveMotionProfile>(moveLimit))
	//.withLinearController(leftController, rightController)
	.build();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize(){
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled(){}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize(){}

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
void autonomous(){
	// TEST FUNCTION
	profiler->setTarget(2_ft);
	chassis->waitUntilSettled();
	profiler->setTarget(90_deg);
	chassis->waitUntilSettled();
	profiler->setTarget(-2_ft);
	chassis->waitUntilSettled();
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
void opcontrol(){
	Controller master(ControllerId::master);
	auto model = chassis->getModel();
	while(true){
		model->curvature(master.getAnalog(ControllerAnalog::leftY), 
						 master.getAnalog(ControllerAnalog::rightX), 
						 0.05);
		pros::delay(10);
	}

}
