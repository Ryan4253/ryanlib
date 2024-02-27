#include "AsyncProfiler.hpp"

namespace ryan{
AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> chassis, 
                                         std::unique_ptr<LinearMotionProfile> profile, 
                                         std::unique_ptr<FFVelocityController> leftController, 
                                         std::unique_ptr<FFVelocityController> rightController,
                                         const okapi::TimeUtil& timeUtil) : 
    chassis(chassis),
    profile(std::move(profile)),
    leftController(std::move(leftController)),
    rightController(std::move(rightController)),
    timeUtil(timeUtil),
    rate(std::move(timeUtil.getRate())),
    timer(std::move(timeUtil.getTimer())),
    leftMotor(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor()),
    rightMotor(std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor())),
    useCustomVelocityController(true){}

AsyncMotionProfiler::AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> chassis, 
                                         std::unique_ptr<LinearMotionProfile> profile, 
                                         const okapi::TimeUtil& timeUtil) : 
    chassis(chassis),
    profile(std::move(profile)),
    timeUtil(timeUtil),
    rate(std::move(timeUtil.getRate())),
    timer(std::move(timeUtil.getTimer())),
    leftMotor(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor()),
    rightMotor(std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor())),
    useCustomVelocityController(false){}

void AsyncMotionProfiler::setConstraint(const ProfileConstraint& iConstraint){
    std::lock_guard<pros::Mutex> lock(mutex);
    resetRobot();
    profile->setConstraint(iConstraint);
}

void AsyncMotionProfiler::setTarget(okapi::QLength distance, bool waitUntilSettled){
    std::lock_guard<pros::Mutex> lock(mutex);
    resetRobot();
    setState(MotionProfileState::MOVE);
    profile->setDistance(distance);
    motionDuration = profile->getTotalTime() + 0.02 * okapi::second;

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(const Trajectory& path, bool waitUntilSettled){
    std::lock_guard<pros::Mutex> lock(mutex);
    resetRobot();
    setState(MotionProfileState::FOLLOW);
    this->path = path;
    motionDuration = (path.size() * 10 + 20) * okapi::millisecond;

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::setTarget(okapi::QAngle angle, bool waitUntilSettled){
    std::lock_guard<pros::Mutex> lock(mutex);
    resetRobot();
    setState(MotionProfileState::TURN);
    profile->setDistance(angle.convert(okapi::radian) * chassis->getChassisScales().wheelTrack / 2);
    motionDuration = profile->getTotalTime() + 0.02 * okapi::second;
    
    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncMotionProfiler::stop(){
    std::lock_guard<pros::Mutex> lock(mutex);
    setState(MotionProfileState::IDLE);
    resetRobot();
}

void AsyncMotionProfiler::loop(){
    using namespace ryan::Math;

    okapi::EmaFilter lFilter(0.7);
    okapi::EmaFilter rFilter(0.7);

    const auto scales = chassis->getChassisScales();
    const auto ratio = chassis->getGearsetRatioPair();
    
    while(true){
        rate->delayUntil(10);
        std::lock_guard<pros::Mutex> lock(mutex);

        const okapi::QTime timeElapsed = timer->getDtFromMark();

        if(timeElapsed > motionDuration){
            setState(MotionProfileState::IDLE);
            chassis->getModel()->tank(0, 0);
        }

        TrajectoryPoint segment;
        switch(getState()){
            case MotionProfileState::IDLE:
                continue;
            
            case MotionProfileState::MOVE:
                segment = profile->get(timeElapsed);
                break;
            
            case MotionProfileState::TURN:
                segment = profile->get(timeElapsed);
                segment.rightPosition = -segment.rightPosition;
                segment.rightVelocity = -segment.rightVelocity;
                segment.rightAcceleration = -segment.rightAcceleration;
                break;

            case MotionProfileState::FOLLOW:
                segment = path[(int)(timeElapsed.convert(okapi::millisecond) / 10)];
                break;

            default:
                continue;
        }

        if(useCustomVelocityController){
            const double leftActualPos = tickToFt(leftMotor->getPosition(), scales, ratio);
            const double leftActualVel = lFilter.filter(rpmToFtps(leftMotor->getActualVelocity(), scales, ratio));
            const double rightActualPos = tickToFt(rightMotor->getPosition(), scales, ratio);
            const double rightActualVel = rFilter.filter(rpmToFtps(rightMotor->getActualVelocity(), scales, ratio));

            const double leftVoltage = leftController->step(
                segment.leftPosition, segment.leftVelocity, segment.leftAcceleration, leftActualPos, leftActualVel);
            const double rightVoltage = rightController->step(
                segment.rightPosition, segment.rightVelocity, segment.rightAcceleration, rightActualPos, rightActualVel);
            
            chassis->getModel()->tank(leftVoltage, rightVoltage);
        }
        else{
            const double leftVel = Math::ftpsToRPM(segment.leftVelocity, scales, ratio);
            const double rightVel = Math::ftpsToRPM(segment.leftVelocity, scales, ratio);

            leftMotor->moveVelocity(leftVel);
            rightMotor->moveVelocity(rightVel);
        }
    }
}

void AsyncMotionProfiler::waitUntilSettled(){
    while(getState() != MotionProfileState::IDLE){
        pros::delay(10);
    }
}

void AsyncMotionProfiler::resetRobot(){
    chassis->getModel()->tank(0, 0);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    timer->placeMark();
}


AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withOutput(std::shared_ptr<okapi::ChassisController> chassis){
    this->chassis = chassis;
    chassisInitialized = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withProfiler(std::unique_ptr<LinearMotionProfile> profile){
    this->profile = std::move(profile);
    profileInitialized = true;
    return *this;
}

AsyncMotionProfilerBuilder& AsyncMotionProfilerBuilder::withController(FFVelocityController leftController, FFVelocityController rightController){
    left = leftController, 
    right = rightController;
    controllerInitialized = true;
    return *this;
}

std::shared_ptr<AsyncMotionProfiler> AsyncMotionProfilerBuilder::build(){
    if(!chassisInitialized){
        throw std::invalid_argument("AsyncMotionProfilerBuilder: Chassis not supplied!");
    }

    if(!profileInitialized){
        throw std::invalid_argument("AsyncMotionProfilerBuilder: Motion profile not supplied!");
    }

    if(!controllerInitialized){
        std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(
            chassis,
            std::move(profile),
            okapi::TimeUtilFactory::createDefault()
        ));

        ret->startTask();
        return ret;
    }

    std::shared_ptr<AsyncMotionProfiler> ret(new AsyncMotionProfiler(
        chassis,
        std::move(profile),
        std::make_unique<FFVelocityController>(left),
        std::make_unique<FFVelocityController>(right),
        okapi::TimeUtilFactory::createDefault()
    ));

    ret->startTask();
    return ret;
}

} // namespace ryan