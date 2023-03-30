#pragma once
#include "pros/adi.hpp"
#include "Units.hpp"

namespace ryan{

/**
 * @brief Wrapper class for vex solenoids which allows more convenient control
 * 
 */
class Solenoid {
    pros::ADIDigitalOut piston;
    bool state;

    public:
    /**
     * @brief Construct a new Solenoid object
     * 
     * @param iPort solinoid port
     * @param initState initial state of the piston
     */
    Solenoid(char iPort, bool initState = false);

    /**
     * @brief Destroys the Solenoid object
     * 
     */
    ~Solenoid() = default;

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

    /**
     * @brief Get the State of the cylinder
     * 
     * @return whether the piston is extended 
     */
    bool getState() const;
};

}