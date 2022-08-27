#pragma once
#include "Units.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include <cmath>
#include <utility>
#include <stdexcept>

namespace ryan{

/**
 * @brief Namespace for all math related functions
 * 
 */
namespace Math{

/**
 * @brief sets the value to zero if the value is less than the deadband
 * 
 * @param value the value to limit
 * @param db deadband limit
 * @return double the value controlled with deadband
 */
double deadband(double value, double db);

/**
 * @brief clamps the value within [mn, mx]. Analogous to std::clamp()
 *        The function works for any data type with an overloaded < and > operator
 * 
 * @tparam T the type to use
 * @param val the value to clamp
 * @param mn minimum value
 * @param mx maximum value
 * @return T the clamped value
 */
template<typename T>
T clamp(T val, T mn, T mx){return std::max(std::min(mx, val), mn);}

/**
 * @brief Converts ft/s to RPM
 * 
 * @param ftps feets per second
 * @return rounds per minute
 */
double ftpsToRPM(double ftps, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset);

/**
 * @brief Converts RPM to ft/s
 * 
 * @param rpm rounds per minute
 * @return feet per second
 */
double rpmToFtps(double rpm, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset);

/**
 * @brief Converts ft to encoder ticks (aka. encoder degrees)
 * 
 * @param ft feet
 * @return encoder ticks 
 */
double ftToTick(double ft, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset);

/**
 * @brief Converts encoder ticks to ft
 * 
 * @param tick encoder ticks
 * @return feet
 */
double tickToFt(double tick, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset);

/**
 * @brief Rescales an angle to [-180, 180]
 * 
 * @param angle Angle to be rescaled
 * @return rescaled angle
 */
okapi::QAngle rescale180(okapi::QAngle angle);

/**
 * @brief Rescales an angle to [-180, 180]
 * 
 * @param angle angle to be converted
 * @return scaled angle to [-180, 180]
 */
double rescale180(double angle);

/**
 * @brief Rescales an angle to [0, 360]
 * 
 * @param angle angle to be rescaled
 * @return rescaled angle
 */
okapi::QAngle rescale360(okapi::QAngle angle);

/**
 * @brief Rescales an angle to [0, 360]
 * 
 * @param angle angle to be converted
 * @return scaled angle to [0, 360]
 */
double rescale360(double angle);

/**
 * @brief The quadratic formula [https://youtu.be/VOXYMRcWbF8]
 *        Computes the two roots of a quadratic equation
 *        x = -b +- sqrt(b^2 - 4ac) / 2a
 * 
 * @param a ^
 * @param b ^
 * @param c ^
 * @return pair containing the two roots, first value is the larger solution
 */
std::pair<double, double> quadraticFormula(double a, double b, double c);

/**
 * @brief returns the sign of the value
 * 
 * @param value the value to check the sign
 * @return int the sign of the value (-1, 0, 1)
 */
int signum(double value);

};

}
