#include "LinearMotionProfile.hpp"

namespace ryan{

// Profile Constraint
ProfileConstraint::ProfileConstraint(okapi::QSpeed maxVel, okapi::QAcceleration maxAccel, okapi::QAcceleration maxDecel, okapi::QJerk maxJerk){
    if(maxVel == 0 * okapi::mps){
        throw std::invalid_argument("ProfileConstraint: Max Velocity can't be zero");
    }
    else if(maxAccel == 0 * okapi::mps2){
        throw std::invalid_argument("ProfileConstraint: Max Acceleration can't be zero");
    }
    else if(maxDecel == 0 * okapi::mps2){
        maxDecel = maxAccel;
    }
    else if(maxJerk == 0 * okapi::mps3){
        throw std::invalid_argument("ProfileConstraint: Max Jerk can't be zero (choose an arbitrary large number if you want a trapezoidal profile)");
    }

    this->maxVelocity = abs(maxVel);
    this->maxAcceleration = abs(maxAccel);
    this->maxDeceleration = abs(maxDecel);
    this->maxJerk = abs(maxJerk);
}

ProfileConstraint::ProfileConstraint(okapi::QSpeed maxVel, okapi::QAcceleration maxAccel, okapi::QJerk maxJerk){
    if(maxVel == 0 * okapi::mps){
        throw std::invalid_argument("ProfileConstraint: Max Velocity can't be zero");
    }
    else if(maxAccel == 0 * okapi::mps2){
        throw std::invalid_argument("ProfileConstraint: Max Acceleration can't be zero");
    }
    else if(maxJerk == 0 * okapi::mps3){
        throw std::invalid_argument("ProfileConstraint: Max Jerk can't be zero (choose an arbitrary large number if you want a trapezoidal profile)");
    }

    this->maxVelocity = abs(maxVel);
    this->maxAcceleration = abs(maxAccel);
    this->maxDeceleration = abs(maxAccel);
    this->maxJerk = abs(maxJerk);
}


// Trapezoidal Motion Profile
TrapezoidalMotionProfile::TrapezoidalMotionProfile(ProfileConstraint iConstraint){
    distPhase.resize(3);
    timePhase.resize(3);
    velPhase.resize(3);
    accPhase.resize(3);
    setConstraint(iConstraint);
}

void TrapezoidalMotionProfile::setConstraint(ProfileConstraint iConstraint){
    constraint = iConstraint;
    min3Stage = constraint.maxVelocity * constraint.maxVelocity / constraint.maxAcceleration / 2 +
                  constraint.maxVelocity * constraint.maxVelocity / constraint.maxDeceleration / 2;
    accPhase = {constraint.maxAcceleration, 0 * okapi::mps2, -constraint.maxAcceleration};
    setDistance(distance);
}

void TrapezoidalMotionProfile::setDistance(okapi::QLength iDistance){
    isReversed = (iDistance < 0 * okapi::meter);
    distance = abs(iDistance);

    if(distance < min3Stage){ // 2 stage
        timePhase[0] = sqrt(distance / constraint.maxAcceleration);
        distPhase[0] = distance / 2;
        velPhase[0] = 0 * okapi::mps;

        timePhase[1] = 0 * okapi::second;
        distPhase[1] = 0 * okapi::meter;
        velPhase[1] = timePhase[0] * constraint.maxAcceleration;

        timePhase[2] = timePhase[0];
        distPhase[2] = distPhase[0];
        velPhase[2] = velPhase[1];
    }
    else{ // full trapezoidal profile
        timePhase[0] = constraint.maxVelocity / constraint.maxAcceleration;
        distPhase[0] = 0.5 * constraint.maxVelocity * constraint.maxVelocity / constraint.maxAcceleration;
        velPhase[0] = 0 * okapi::mps;

        timePhase[1] = (distance - distPhase[0] * 2) / constraint.maxVelocity;
        distPhase[1] = constraint.maxVelocity * timePhase[1];
        velPhase[1] = constraint.maxVelocity;

        timePhase[2] = timePhase[0];
        distPhase[2] = distPhase[0];
        velPhase[2] = velPhase[1];
    }

    for(int i = 1; i < 3; i++){
        timePhase[i] += timePhase[i-1];
        distPhase[i] += distPhase[i-1];
    }
}

okapi::QTime TrapezoidalMotionProfile::getTotalTime() const{
    return timePhase[2];
}

okapi::QSpeed TrapezoidalMotionProfile::getVelocity(okapi::QTime time) const{
    okapi::QSpeed ret;
    if(time < 0 * okapi::second || time > timePhase[2]){
        ret = 0 * okapi::mps;
    }
    else if(time < timePhase[0]){
        ret = constraint.maxAcceleration * time;
    }
    else if(time > timePhase[1]){
        ret = constraint.maxAcceleration * (timePhase[2] - time);
    }
    else{
        ret = constraint.maxVelocity;
    }

    return isReversed ? -ret : ret;
}

okapi::QAcceleration TrapezoidalMotionProfile::getAcceleration(okapi::QTime time) const{
    okapi::QAcceleration ret;
    if(time < 0 * okapi::second || time > timePhase[2]){
        ret = 0 * okapi::mps2;
    }
    else if(time < timePhase[0]){
        ret = constraint.maxAcceleration;
    }
    else if(time > timePhase[1]){
        ret = -1 * constraint.maxAcceleration;
    }
    else{
        ret = 0 * okapi::mps2;
    }

    return isReversed ? -1 * ret : ret;
}

okapi::QLength TrapezoidalMotionProfile::getPosition(okapi::QTime time) const{
    okapi::QLength ret;
    if(time < 0 * okapi::second){
        ret = 0 * okapi::meter;
    }
    else if(time > timePhase[2]){
        ret = distance;
    }
    else if(time < timePhase[0]){
        ret = 0.5 * constraint.maxAcceleration * time * time;
    }
    else if(time > timePhase[1]){
        okapi::QTime dTime = time - timePhase[1];
        ret = distPhase[1] + velPhase[2] * dTime + 0.5 * accPhase[2] * dTime * dTime;
    }
    else{
        ret = distPhase[0] + constraint.maxVelocity * (time - timePhase[0]);
    }

    return isReversed ? -1 * ret : ret;
}

TrajectoryPoint TrapezoidalMotionProfile::get(okapi::QTime time) const{
    double position = getPosition(time).convert(okapi::foot);
    double velocity = getVelocity(time).convert(okapi::ftps);
    double acceleration = getAcceleration(time).convert(okapi::ftps2);

    return {position, position, velocity, velocity, acceleration, acceleration};
}

// S Curve Motion Profile
SCurveMotionProfile::SCurveMotionProfile(ProfileConstraint iConstraint){
    timePhase.resize(7);
    distPhase.resize(7);
    velPhase.resize(7);
    accPhase.resize(7);
    setConstraint(iConstraint);
}

void SCurveMotionProfile::setConstraint(ProfileConstraint iConstraint){
    constraint = iConstraint;  
    jerkPhase = {iConstraint.maxJerk, 0 * okapi::mps3, -iConstraint.maxJerk, 0 * okapi::mps3, -iConstraint.maxJerk, 0 * okapi::mps3, iConstraint.maxJerk};

    okapi::QTime time = constraint.maxAcceleration / constraint.maxJerk;
    if(constraint.maxJerk * time * time >= constraint.maxVelocity){
        fullAccel = false;
        okapi::QTime t1 = sqrt(constraint.maxVelocity / constraint.maxJerk);
        minDist = constraint.maxJerk * t1 * t1 * t1 * 2;
        fullDist = minDist;
    }
    else{
        fullAccel = true;
        okapi::QTime t1 = constraint.maxAcceleration / constraint.maxJerk;
        minDist = constraint.maxJerk * t1 * t1 * t1 * 2;

        okapi::QTime t2 = (constraint.maxVelocity - (constraint.maxJerk * t1 * t1)) / constraint.maxAcceleration;
        fullDist = (0.5 * constraint.maxJerk * t1 * t1) * t2 + 0.5 * (constraint.maxAcceleration) * t2 * t2;
        fullDist += constraint.maxVelocity * t1;
        fullDist *= 2;
    }

    setDistance(distance);
}

void SCurveMotionProfile::setDistance(okapi::QLength iDistance){
    isReversed = iDistance < 0 * okapi::meter ;
    distance = abs(iDistance);

    if(distance < minDist){ // 4 stage
        timePhase[1] = timePhase[3] = timePhase[5] = 0 * okapi::second;
        timePhase[0] = timePhase[2] = timePhase[4] = timePhase[6] = cbrt(distance / constraint.maxJerk / 2);

        distPhase[0] = distPhase[6] = timePhase[0] * timePhase[0] * timePhase[0] * constraint.maxJerk / 6;
        distPhase[1] = distPhase[3] = distPhase[5] = 0 * okapi::meter;
        distPhase[2] = distPhase[4] = 0.5 * distance - distPhase[0];
        
        velPhase[0] = 0 * okapi::mps;
        velPhase[1] = velPhase[2] = velPhase[5] = velPhase[6] = 0.5 * constraint.maxJerk * timePhase[0] * timePhase[0]; 
        velPhase[3] = velPhase[4] = velPhase[1] * 2;
        
        accPhase[0] = accPhase[3] = accPhase[4] = 0 * okapi::mps2;
        accPhase[1] = accPhase[2] = constraint.maxJerk * timePhase[0];
        accPhase[5] = accPhase[6] = -accPhase[1];
    }
    else if(!fullAccel){ // 5 stage
        timePhase[0] = timePhase[2] = timePhase[4] = timePhase[6] = sqrt(constraint.maxVelocity / constraint.maxJerk);
        timePhase[1] = timePhase[5] = 0 * okapi::second;
        timePhase[3] = (distance - constraint.maxVelocity * timePhase[0] * 2) / constraint.maxVelocity;

        distPhase[0] = distPhase[6] = timePhase[0] * timePhase[0] * timePhase[0] * constraint.maxJerk / 6;
        distPhase[1] = distPhase[5] = 0 * okapi::meter;
        distPhase[2] = distPhase[4] = constraint.maxVelocity * timePhase[0] - distPhase[0];
        distPhase[3] = distance - distPhase[0] * 2 - distPhase[2] * 2;

        velPhase[0] = 0 * okapi::mps;
        velPhase[1] = velPhase[2] = velPhase[5] = velPhase[6] = constraint.maxVelocity / 2;
        velPhase[3] = velPhase[4] = constraint.maxVelocity;

        accPhase[0] = accPhase[3] = accPhase[4] = 0 * okapi::mps2;
        accPhase[1] = accPhase[2] = constraint.maxJerk * timePhase[0];
        accPhase[5] = accPhase[6] = -accPhase[1];
    }
    else if(distance < fullDist){ // 6 stage
        double a = (constraint.maxAcceleration).convert(okapi::ftps2);
        double b = (3 * constraint.maxAcceleration * constraint.maxAcceleration / constraint.maxJerk).convert(okapi::ftps);
        double c = (2 * constraint.maxAcceleration * constraint.maxAcceleration * constraint.maxAcceleration / constraint.maxJerk / constraint.maxJerk - distance).convert(okapi::foot);
        auto t2Candidate = ryan::Math::quadraticFormula(a, b, c);

        timePhase[0] = timePhase[2] = timePhase[4] = timePhase[6] = constraint.maxAcceleration / constraint.maxJerk;
        timePhase[1] = timePhase[5] = std::max(t2Candidate.first, t2Candidate.second) * okapi::second;
        timePhase[3] = 0 * okapi::second;

        accPhase[0] = accPhase[3] = accPhase[4] = 0 * okapi::mps2;
        accPhase[1] = accPhase[2] = constraint.maxJerk * timePhase[0];
        accPhase[5] = accPhase[6] = -accPhase[1];

        velPhase[0] = 0 * okapi::mps;
        velPhase[1] = velPhase[6] = 0.5 * constraint.maxJerk * timePhase[0] * timePhase[0];
        velPhase[2] = velPhase[5] = velPhase[1] + constraint.maxAcceleration * timePhase[1];
        velPhase[3] = velPhase[4] = velPhase[2] + accPhase[2] * timePhase[2] - 0.5 * constraint.maxJerk * timePhase[2] * timePhase[2]; 

        distPhase[0] = distPhase[6] = constraint.maxJerk * timePhase[0] * timePhase[0] * timePhase[0] / 6;
        distPhase[1] = distPhase[5] = velPhase[1] * timePhase[1] + 0.5 * accPhase[1] * timePhase[1] * timePhase[1];
        distPhase[2] = distPhase[4] = velPhase[2] * timePhase[2] + 0.5 * accPhase[2] * timePhase[2] * timePhase[2] - constraint.maxJerk * timePhase[2] * timePhase[2] * timePhase[2] / 6;
        distPhase[3] = 0 * okapi::meter;
    }
    else{ // full s curve
        velPhase[0] = 0 * okapi::mps;
        accPhase[0] = 0 * okapi::mps2;
        timePhase[0] = constraint.maxAcceleration / constraint.maxJerk;
        distPhase[0] = constraint.maxJerk * timePhase[0] * timePhase[0] * timePhase[0] / 6;
        
        velPhase[1] = 0.5 * constraint.maxJerk * timePhase[0] * timePhase[0];
        accPhase[1] = constraint.maxAcceleration;
        timePhase[1] = (constraint.maxVelocity - velPhase[1]*2) / constraint.maxAcceleration;
        distPhase[1] = velPhase[1] * timePhase[1] + 0.5 * accPhase[1] * timePhase[1] * timePhase[1];

        velPhase[2] = velPhase[1] + accPhase[1] * timePhase[1];
        accPhase[2] = constraint.maxAcceleration;
        timePhase[2] = timePhase[0];
        distPhase[2] = velPhase[2] * timePhase[2] + 0.5 * accPhase[2] * timePhase[2] * timePhase[2] - constraint.maxJerk * timePhase[2] * timePhase[2] * timePhase[2] / 6;

        velPhase[3] = constraint.maxVelocity;
        accPhase[3] = 0 * okapi::mps2;
        timePhase[3] = (distance - 2 * (distPhase[0] + distPhase[1] + distPhase[2])) / constraint.maxVelocity; 
        distPhase[3] = velPhase[3] * timePhase[3];

        velPhase[4] = constraint.maxVelocity;
        accPhase[4] = 0 * okapi::mps2;
        timePhase[4] = timePhase[2];
        distPhase[4] = distPhase[2];

        velPhase[5] = velPhase[2];
        accPhase[5] = -constraint.maxAcceleration;
        timePhase[5] = timePhase[1];
        distPhase[5] = distPhase[1];

        velPhase[6] = velPhase[1];
        accPhase[6] = -constraint.maxAcceleration;
        timePhase[6] = timePhase[0];
        distPhase[6] = distPhase[0];
    }

    for(int i = 1; i < 7; i++){
        timePhase[i] += timePhase[i-1];
        distPhase[i] += distPhase[i-1];
    }
}

okapi::QTime SCurveMotionProfile::getTotalTime() const{
    return timePhase[6];
}

okapi::QLength SCurveMotionProfile::getPosition(okapi::QTime time) const{
    okapi::QLength pos = 0 * okapi::meter;
    
    if(time < 0 * okapi::second){
        pos = 0 * okapi::meter;
    }
    else if(time >= timePhase[6]){
        pos = distance;
    }
    else if(time < timePhase[0]){
        pos = constraint.maxJerk * time * time * time / 6;
    }
    else{
        int index = 1;
        for(int i = 1; i < 7; i++){
            if(time < timePhase[i]){
                index = i;
                break;
            }
        }
        okapi::QTime dTime = time - timePhase[index-1];
        pos = distPhase[index-1] + velPhase[index] * dTime + 0.5 * accPhase[index] * dTime * dTime + jerkPhase[index] * dTime * dTime * dTime / 6;
    }

    return isReversed ? -pos : pos; 
}

okapi::QSpeed SCurveMotionProfile::getVelocity(okapi::QTime time) const{
    okapi::QSpeed vel = 0 * okapi::mps;
    
    if(time < 0 * okapi::second || time >= timePhase[6]){
        vel = 0 * okapi::mps;
    }
    else if(time < timePhase[0]){
        vel = 0.5 * constraint.maxJerk * time * time;
    }
    else{
        int index = 1;
        for(int i = 1; i < 7; i++){
            if(time < timePhase[i]){
                index = i;
                break;
            }
        }
        okapi::QTime dTime = time - timePhase[index-1];
        vel = velPhase[index] + accPhase[index] * dTime + jerkPhase[index] * dTime * dTime * 0.5;
    }

    return isReversed ? -vel : vel;  
}

okapi::QAcceleration SCurveMotionProfile::getAcceleration(okapi::QTime time) const{
    okapi::QAcceleration acc = 0 * okapi::mps2;
    
    if(time < 0 * okapi::second || time >= timePhase[6]){
        acc = 0 * okapi::mps2;
    }
    else if(time < timePhase[0]){
        acc = constraint.maxJerk * time;
    }
    else{
        int index = 1;
        for(int i = 1; i < 7; i++){
            if(time < timePhase[i]){
                index = i;
                break;
            }
        }
        okapi::QTime dTime = time - timePhase[index-1];
        acc = accPhase[index] + jerkPhase[index] * dTime;
    }

    return isReversed ? -acc : acc;  
}

TrajectoryPoint SCurveMotionProfile::get(okapi::QTime time) const{
    double position = getPosition(time).convert(okapi::foot);
    double velocity = getVelocity(time).convert(okapi::ftps);
    double acceleration = getAcceleration(time).convert(okapi::ftps2);

    return {position, position, velocity, velocity, acceleration, acceleration};
}

}