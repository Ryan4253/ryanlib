#include "main.h"

// change the drive's max velocity, acceleration, and jerk
// just tune until its fast and the robot doesnt jerk upwards
ProfileConstraint constraint({4.5_ftps, 9_ftps2, 15_ftps3});

// for withMotor(), change the motor port, negative if reversed
// for withDimension(), enter cartridge, ratio (wheel gear / motor gear), wheel diameter, track diameter, cartridge tpr
// the example below is used for a 6m 360 3.25" drive
auto chassis = ChassisControllerBuilder()
	.withMotors({1, -2, 3}, {4, -5, 6})
	.withDimensions({AbstractMotor::gearset::blue, 60.0/36.0}, {{3.25_in, 10.25_in}, imev5BlueTPR})
	.build();

// you can decide to add your custom velocity controller with the withLinearController() and withTrajectoryController()
// the built in control will most likely be very precise, but probably not very accurate
// if you use TrapezoidalMotionProfile, you can also specify a separete deceleration (that will prob break when going backwards)
auto profiler = AsyncMotionProfilerBuilder()
	.withOutput(chassis)
	.withProfiler(std::make_unique<SCurveMotionProfile>(constraint))
	.build();

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::delay(10);
	}
}
