#include "Solenoid.hpp"

namespace ryan{

Solenoid::Solenoid(char iPort, bool initState):piston(iPort), state(initState){
    piston.set_value(state);
}

void Solenoid::toggle(){
    state = !state;
    piston.set_value(state);
}

void Solenoid::set(bool iState){
    state = iState;
    piston.set_value(iState);
}

bool Solenoid::getState() const{
    return state;
}

}