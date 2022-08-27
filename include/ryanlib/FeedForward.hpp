#pragma once
#include "Math.hpp"

namespace ryan{

/**
 * @brief Our custom feedforward velocity controller class, can be used when target velocity and acceleration are known
 *        For example, this can be used in following motion profiles, as well as in pure pursuit
 * 
 */
class FFVelocityController{
    private:
        double kV, kA_Up, kA_Down, kP_Pos, kP_Vel;
        double currPos, currVel, power;

    public:
        /**
         * @brief Constructs a new FFVelocityController object
         * 
         */
        FFVelocityController() = default;

        /**
         * @brief Destroys the FFVelocityController object
         * 
         */
        ~FFVelocityController() = default;

        /**
         * @brief Constructs a new FFVelocityController object
         * 
         * @param ikV velocity constant
         * @param ikAU acceleration constant
         * @param ikAD deceleration constant
         * @param ikP_Pos proportional constant for position
         * @param ikP_Vel proportional constant for velocity
         */
        FFVelocityController(double ikV, double ikAU, double ikAD, double ikP_Pos, double ikP_Vel);

        /**
         * @brief Calculates one step in accordance to the given parameters
         * 
         * @param position desired position
         * @param velocity desired velocity
         * @param acceleration desired acceleration
         * @param currentPos current position, in feets
         * @param currentVel current velocity, in ftps
         * @return calculated power to reach the target velocity
         */
        double step(double position, double velocity, double acceleration, double currentPos, double currentVel);

        /**
         * @brief Returns the target power
         * 
         * @return target power
         */
        double getTargetPower() const;

        /**
         * @brief Returns kV constant
         * 
         * @return kV
         */
        double getkV() const;
};

}

