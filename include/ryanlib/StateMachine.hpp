#pragma once
#include "pros/rtos.hpp"

namespace ryan{

/**
 * @brief Template Wrapper class which allows easy state machine (有限狀態機) implementation. 
 *        state machine is an elegant way of programming a subsystem. It involves several
 *        possible "states" the subsystem can be in, and allows the state to automatically
 *        transition into other states to create complex streams of control.
 * 
 *        This specific implementation is thread safe and prevents race condition (競爭危害)
 *        through the use of mutexes
 * 
 * @tparam State the enum class that contains all the possible states of this state machine
 * @tparam initState the initial state of the state machine, assumed to be idle
 */
template<typename State, State initState = State::IDLE>
class StateMachine{
    private:
    State state = initState;
    pros::Mutex stateLock;

    public:
    /**
     * @brief Construct a new State Machine object
     * 
     */
    StateMachine(){}

    /**
     * @brief Get the current state
     * 
     * @return State the current state
     */
    State getState(){
        stateLock.take();
        State currentState = state;
        stateLock.give();
        return currentState;
    }

    /**
     * @brief Set the current state
     * 
     * @param iState the new state
     */
    void setState(State iState){
        stateLock.take();
        state = iState;
        stateLock.give();
    }
};

}