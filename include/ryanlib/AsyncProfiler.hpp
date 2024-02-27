#pragma once
#include "TaskWrapper.hpp"
#include "Trajectory.hpp"
#include "StateMachine.hpp"
#include "FeedForward.hpp"
#include "LinearMotionProfile.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/api/filter/emaFilter.hpp"

namespace ryan{

/**
 * @brief an enum containing all possible states for our motion profile controller
 * 
 */
enum class MotionProfileState{
    MOVE, FOLLOW, TURN, IDLE
};

// forward declare
template class StateMachine<MotionProfileState>;

/**
 * @brief class that allows us to control our chassis asynchronously using motion profiles
 * 
 */
class AsyncMotionProfiler : public StateMachine<MotionProfileState, MotionProfileState::IDLE>, public TaskWrapper {
    protected:
    /**
     * @brief Construct a new Async Motion Profiler object (using all custom velocity control)
     * 
     * @param chassis chassis to output to
     * @param profile linear motion profile constraint to generate
     * @param leftController left velocity controller 
     * @param rightController right velocity controller 
     * @param timeUtil timer utility
     */
    AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> chassis, 
                        std::unique_ptr<LinearMotionProfile> profile, 
                        std::unique_ptr<FFVelocityController> leftController, 
                        std::unique_ptr<FFVelocityController> rightController,
                        const okapi::TimeUtil& timeUtil);

    /**
     * @brief Construct a new Async Motion Profiler object (using internal velocity control)
     * 
     * @param chassis chassis to output to
     * @param profile linear motion profile constraint to generate
     * @param timeUtil timer utility
     */
    AsyncMotionProfiler(std::shared_ptr<okapi::ChassisController> chassis, 
                    std::unique_ptr<LinearMotionProfile> profile, 
                    const okapi::TimeUtil& timeUtil);

    void operator=(const AsyncMotionProfiler& rhs) = delete;

    /**
     * @brief generate this class using AsyncMotionProfilerBuilder only
     * 
     */
    friend class AsyncMotionProfilerBuilder;

    public:

    /**
     * @brief sets the motion profile constraints. Note that any movement currently running will be interrupted
     * 
     * @param consraint the kinematic constraints for the motion profile
     */
    void setConstraint(const ProfileConstraint& consraint);

    /**
     * @brief Set the target distance to move
     * 
     * @param distance target distance
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */
    void setTarget(okapi::QLength distance, bool waitUntilSettled = false);

    /**
     * @brief Set the target trajectory to follow
     * 
     * @param path target trajectory to follow
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(const Trajectory& path, bool waitUntilSettled = false);

    /**
     * @brief Set the target angle to turn
     * 
     * @param angle target angle to turn
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(okapi::QAngle angle, bool waitUntilSettled = false);

    /**
     * @brief stop the chassis from moving
     * 
     */
    void stop();

    /**
     * @brief blocks the current movement until the current movement is complete
     * 
     */
    void waitUntilSettled();

    protected:
    std::shared_ptr<okapi::ChassisController> chassis;
    std::shared_ptr<okapi::AbstractMotor> leftMotor;
    std::shared_ptr<okapi::AbstractMotor> rightMotor;

    std::unique_ptr<LinearMotionProfile> profile;
    std::unique_ptr<FFVelocityController> leftController{nullptr};
    std::unique_ptr<FFVelocityController> rightController{nullptr};

    okapi::TimeUtil timeUtil;
    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;
    okapi::QTime motionDuration{0.0};

    Trajectory path;
    pros::Mutex mutex;

    bool useCustomVelocityController{false};

    /**
     * @brief task loop
     * 
     */
    void loop() override;

    /**
     *  @brief resets robot sensors and motor for new movement
     * 
    */
    void resetRobot();
};


/**
 * @brief An AsyncMotionProfile builder class which allows more intuitive instantiation of the class
 * 
 */
class AsyncMotionProfilerBuilder{
    public:
    /**
     * @brief Constructs a new Async Motion Profiler Builder object
     * 
     */
    AsyncMotionProfilerBuilder() = default;

    /**
     * @brief Destroys the Async Motion Profiler Builder object
     * 
     */
    ~AsyncMotionProfilerBuilder() = default;

    /**
     * @brief sets the chassis object for the profiler to output to
     * 
     * @param chassis the chassis object to output to
     * @return AsyncMotionProfilerBuilder& an ongoing builder
     */
    AsyncMotionProfilerBuilder& withOutput(std::shared_ptr<okapi::ChassisController> chassis);

    /**
     * @brief sets the motion profile generator to use for linear movements
     * 
     * @param profiler the profile generator
     * @return AsyncMotionProfilerBuilder& 
     */
    AsyncMotionProfilerBuilder& withProfiler(std::unique_ptr<LinearMotionProfile> profile);

    /**
     * @brief sets the motor controller to use
     * 
     * @param leftController 
     * @param rightController 
     * @return AsyncMotionProfilerBuilder& 
     */
    AsyncMotionProfilerBuilder& withController(FFVelocityController leftController, FFVelocityController rightController);

    /**
     * @brief builds the async motion profiler object with the specified parameters. The task is started automaically
     * 
     * @return std::shared_ptr<AsyncMotionProfiler> the built async motion profiler
     */
    std::shared_ptr<AsyncMotionProfiler> build();

    private:
    std::unique_ptr<LinearMotionProfile> profile{nullptr};
    std::shared_ptr<okapi::ChassisController> chassis{nullptr};
    FFVelocityController left;
    FFVelocityController right;

    bool chassisInitialized{false};;
    bool profileInitialized{false};
    bool controllerInitialized{false};
};

} // namespace ryan