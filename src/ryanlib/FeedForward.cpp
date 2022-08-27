#include "FeedForward.hpp"

namespace ryan{

FFVelocityController::FFVelocityController(double ikV, double ikAU, double ikAD, double ikP_Pos, double ikP_Vel){
    kV = ikV, kA_Up = ikAU, kA_Down = ikAD, kP_Pos = ikP_Pos, kP_Vel = ikP_Vel;
}

double FFVelocityController::step(double position, double velocity, double acceleration, double currentPos, double currentVel){
    if(acceleration > 0){
        return power = (kV * velocity + kA_Up * acceleration + kP_Pos * (position - currentPos) + kP_Vel * (velocity - currentVel));
    }
    else{
        return power = (kV * velocity + kA_Down * acceleration + kP_Pos * (position - currentPos) + kP_Vel * (velocity - currentVel));
    }
}

double FFVelocityController::getTargetPower() const{
    return power;
}

double FFVelocityController::getkV() const{
    return kV;
}

}

