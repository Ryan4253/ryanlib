#include "AsyncProfiler.hpp"

namespace ryan{
AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                                         std::unique_ptr<LinearMotionProfile> iMove, 
                                         std::unique_ptr<FFVelocityController> iLeftLinear, 
                                         std::unique_ptr<FFVelocityController> iRightLinear,
                                         std::unique_ptr<FFVelocityController> iLeftTrajectory,
                                         std::unique_ptr<FFVelocityController> iRightTrajectory,
                                         const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    leftLinear =  std::move(iLeftLinear);
    rightLinear = std::move(iRightLinear);
    leftTrajectory = std::move(iLeftTrajectory);
    rightTrajectory = std::move(iRightTrajectory);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());
    linearCustom = true;
    trajectoryCustom = true;

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
    std::unique_ptr<LinearMotionProfile> iMove, 
    const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> iChassis, 
                std::unique_ptr<LinearMotionProfile> iMove, 
                std::unique_ptr<FFVelocityController> iLeft,
                std::unique_ptr<FFVelocityController> iRight,
                bool velFlag,
                const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil){
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    if(velFlag){
        leftLinear = std::move(iLeft);
        rightLinear = std::move(iRight);
        linearCustom = true;
    }
    else{
        leftTrajectory = std::move(iLeft);
        rightTrajectory = std::move(iRight);
        trajectoryCustom = true;
    }
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());
}

void AsyncMotionProfiler::setConstraint(const ProfileConstraint& iConstraint){
    stop();
    lock.take(5);
    profiler->setConstraint(iConstraint);
    lock.give();
}

void AsyncMotionProfiler::setTarget(okapi::QLength distance, bool waitUntilSettled){
    lock.take(5);
    setState(MotionProfileState::MOVE);
    profiler->setDistance(distance);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    maxTime = profiler->getTotalTime() + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(const Trajectory& iPath, bool waitUntilSettled){
    lock.take(5);
    setState(MotionProfileState::FOLLOW);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    path = iPath;
    maxTime = path.size() * 10 * okapi::millisecond + 0.02 * okapi::millisecond;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(okapi::QAngle iAngle, bool waitUntilSettled){
    lock.take(5);
    setState(MotionProfileState::TURN);
    profiler->setDistance(iAngle.convert(okapi::radian) * chassis->getChassisScales().wheelTrack/2);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    maxTime = profiler->getTotalTime() + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();
    
    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::stop(){
    lock.take(5);
    setState(MotionProfileState::IDLE);
    (chassis->getModel())->tank(0, 0);
    lock.give();
}

void AsyncMotionProfiler::loop(){
    TrajectoryPoint pt;
    double leftPower, rightPower;
    okapi::EmaFilter lFilter(0.7);
    okapi::EmaFilter rFilter(0.7);

    while(true){
        lock.take(5);
        okapi::QTime time = timer->getDtFromMark();

        double leftPos = Math::tickToFt(leftMotor->getPosition(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double leftVel = Math::rpmToFtps(leftMotor->getActualVelocity(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double rightPos = Math::tickToFt(rightMotor->getPosition(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double rightVel = Math::rpmToFtps(rightMotor->getActualVelocity(), chassis->getChassisScales(), chassis->getGearsetRatioPair());

        std::cout << lFilter.filter(leftVel) << std::endl;
        //std::cout << rFilter.filter(rightVel) << std::endl;
        //std::cout << leftPos << std::endl;
        //std::cout << rightPos << std::endl;

        if(getState() == MotionProfileState::IDLE){

        }
        else if(time > maxTime){
            setState(MotionProfileState::IDLE);
            chassis->getModel()->tank(0, 0);
        }
        else if(getState() == MotionProfileState::MOVE){
            pt = profiler->get(time);
            if(linearCustom){
                leftPower = leftLinear->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightLinear->step(pt.rightPosition, pt.rightVelocity, pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double vel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(vel);
                rightMotor->moveVelocity(vel);
            }
        }
        else if(getState() == MotionProfileState::TURN){
            pt = profiler->get(time);
            if(linearCustom){
                leftPower = leftLinear->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightLinear->step(-pt.rightPosition, -pt.rightVelocity, -pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double vel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(vel);
                rightMotor->moveVelocity(-vel);
            }
        }
        else if(getState() == MotionProfileState::FOLLOW){
            pt = path[(int)(time.convert(okapi::millisecond) / 10)];
            if(trajectoryCustom){
                leftPower = leftTrajectory->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightTrajectory->step(pt.rightPosition, pt.rightVelocity, pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double leftVel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                double rightVel = Math::ftpsToRPM(pt.rightVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(leftVel);
                rightMotor->moveVelocity(rightVel);
            }
        }

        lock.give();
        rate->delayUntil(10);
    }
}

void AsyncMotionProfiler::waitUntilSettled(){
    while(getState() != MotionProfileState::IDLE){
        pros::delay(10);
    }
}

AsyncMotionProfilerBuilder::AsyncMotionProfilerBuilder(){
    linearInit = false;
    trajInit = false;
    driveInit = false;
    profileInit = false;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withOutput(std::shared_ptr<okapi::ChassisController> iChassis){
    chassis = std::move(iChassis);
    driveInit = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler){
    profile = std::move(iProfiler);
    profileInit = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withLinearController(FFVelocityController iLeft, FFVelocityController iRight){
    leftL = iLeft, rightL = iRight;
    linearInit = true;
    return *this;

}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withTrajectoryController(FFVelocityController iLeft, FFVelocityController iRight){
    leftT = iLeft, rightT = iRight;
    trajInit = true;
    return *this;
}

std::shared_ptr<AsyncMotionProfiler> AsyncMotionProfilerBuilder::build(){
    if(driveInit && profileInit && linearInit && trajInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                            std::move(profile), 
                                            std::make_unique<FFVelocityController>(leftL), 
                                            std::make_unique<FFVelocityController>(rightL),
                                            std::make_unique<FFVelocityController>(leftT), 
                                            std::make_unique<FFVelocityController>(rightT),
                                            okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit && linearInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<FFVelocityController>(leftL), 
                                    std::make_unique<FFVelocityController>(rightL),
                                    true,
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit && trajInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<FFVelocityController>(leftT), 
                                    std::make_unique<FFVelocityController>(rightT),
                                    false,
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else if(driveInit && profileInit){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        return std::move(ret);
    }
    else{
        throw std::runtime_error("AsyncMotionProfilerBuilder: Not all parameters supplied, failed to build (you need at least a chassis and a profiler");
    }
}

}







