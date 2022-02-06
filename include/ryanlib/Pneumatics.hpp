#pragma once
#include "pros/adi.hpp"

namespace ryan{

/**
 * @brief Wrapper class for vex Pneumatics which allows more convenient control
 * 
 */
class Pneumatics {
    pros::ADIDigitalOut piston;
    bool state;

    public:
    /**
     * @brief Construct a new Pneumatics object
     * 
     * @param iPort solinoid port
     * @param initState initial state of the piston
     */
    Pneumatics(char iPort, bool initState = false);

    /**
     * @brief Destroys the Pneumatics object
     * 
     */
    ~Pneumatics() = default;

    /**
     * @brief Toggles the solinoid to the value opposite to the current state
     * 
     */
    void toggle();

    /**
     * @brief Sets the state of the solinoid
     * 
     * @param iState state
     */
    void set(bool iState);
};

}