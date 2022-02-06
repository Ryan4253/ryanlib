#pragma once
#include "Units.hpp"
#include "Math.hpp"
#include "Trajectory.hpp"

namespace ryan{

/**
 * @brief A struct that stores the physical constraints of the chassis
 * 
 */
struct ProfileConstraint{
    okapi::QSpeed maxVelocity{0.0};
    okapi::QAcceleration maxAcceleration{0.0};
    okapi::QAcceleration maxDeceleration{0.0};
    okapi::QJerk maxJerk{0.0};

    /**
     * @brief Constructs a new Profile Constraint object
     * 
     * @param maxVel max velocity
     * @param maxAccel max acceleration
     * @param maxDecel max deceleration (only used in trapezoidal motion profiles)
     * @param maxJerk max jerk (only used in s curve motion profiles)
     */
    ProfileConstraint(okapi::QSpeed maxVel, okapi::QAcceleration maxAccel, okapi::QAcceleration maxDecel, okapi::QJerk maxJerk);

    ProfileConstraint(okapi::QSpeed maxVel, okapi::QAcceleration maxAccel, okapi::QJerk maxJerk);

    /**
     * @brief Destroys the Profile Constraint object
     * 
     */
    ~ProfileConstraint() = default;

    protected:
    /**
     * @brief Constructs a Profile Constraint object - only accessible by the LinearMotionProfile class
     * 
     */
    ProfileConstraint() = default;

    /**
     * @brief forward declared to be friend
     * 
     */
    friend class LinearMotionProfile;
};

/**
 * @brief An abstract class which acts as a base of all linear motion profile classes
 * 
 */
class LinearMotionProfile {
    protected:
    ProfileConstraint constraint;
    okapi::QLength distance{0.0};
    bool isReversed{false};

    std::vector<okapi::QTime> timePhase;
    std::vector<okapi::QLength> distPhase;
    std::vector<okapi::QSpeed> velPhase;
    std::vector<okapi::QAcceleration> accPhase;
    std::vector<okapi::QJerk> jerkPhase;

    public:
    /**
     * @brief Constructs a new Linear Motion Profile object
     * 
     */
    LinearMotionProfile() = default;

    /**
     * @brief Destroys the Linear Motion Profile object
     * 
     */
    ~LinearMotionProfile() = default;

    /**
     * @brief sets the target distance 
     * 
     * @param iDistance new target distance
     */
    virtual void setDistance(okapi::QLength iDistance) = 0;

    /**
     * @brief sets chassis constraints
     * 
     * @param iConstraint new constraint
     */
    virtual void setConstraint(ProfileConstraint iConstraint) = 0;

    /**
     * @brief Gets the total time to run the profile
     * 
     * @return QTime total time to run the profile
     */
    virtual okapi::QTime getTotalTime() const = 0;

    /**
     * @brief Gets the target position at a given time
     * 
     * @param time time step to query
     * @return QLength the distance travelled at the target time
     */
    virtual okapi::QLength getPosition(okapi::QTime time) const = 0;

    /**
     * @brief Gets the target velocity at a given time
     * 
     * @param time time step to query
     * @return QSpeed target velocity at the target time
     */
    virtual okapi::QSpeed getVelocity(okapi::QTime time) const = 0;

    /**
     * @brief Gets the target acceleration at a given time
     * 
     * @param time time step to query
     * @return QAcceleration target acceleration at the target time
     */
    virtual okapi::QAcceleration getAcceleration(okapi::QTime time) const = 0;
    
    /**
     * @brief gets the kinematics data (position, velocity, acceleration) at a given time
     * 
     * @param time time step to query
     * @return TrajectoryPoint target kinematics data at the target time
     */
    virtual TrajectoryPoint get(okapi::QTime time) const = 0;
};

/**
 * @brief class which generates "trapezoidal linear motion profiles" for the chassis to follow.
 *        Given a target distance, it generates a set of velocities which makes the chassis
 *        accelerate at max acceleration, cruise at max velocity, then decelerate at max
 *        deceleration. As a result, the velocity curve of the chassis' translation looks like
 *        a trapezoid. 
 * 
 *       Since all the methods are inherited from the LinearMotionProfile class, repeated comments
 *       are omitted. 
 */
class TrapezoidalMotionProfile : public LinearMotionProfile{
    private:
    okapi::QLength min3Stage = 0 * okapi::meter;

    public:
    TrapezoidalMotionProfile(ProfileConstraint iConstraint);
    TrapezoidalMotionProfile() = default;
    ~TrapezoidalMotionProfile() = default;

    void setDistance(okapi::QLength iDistance) override;
    void setConstraint(ProfileConstraint iConstraint) override;

    okapi::QTime getTotalTime() const override;
    okapi::QLength getPosition(okapi::QTime time) const override;
    okapi::QSpeed getVelocity(okapi::QTime time) const override;
    okapi::QAcceleration getAcceleration(okapi::QTime time) const override;
    TrajectoryPoint get(okapi::QTime time) const override;
};

/**
 * @brief class which generates "s curve linear motion profiles" for the chassis to follow
 *        Given a target distance, it generates a set of velocities which makes the chassis
 *        move at max jerk, max acceleration, max velocity, and decelerate the same way. 
 *        As a result, the velocity curve of the chassis' translation looks like
 *        a smooth s shaped trapezoid. 
 * 
 *        Since all the methods are inherited from the LinearMotionProfile class, repeated comments
 *        are omitted. 
 */
class SCurveMotionProfile : public LinearMotionProfile{
    okapi::QLength fullDist = 0 * okapi::meter;
    okapi::QLength minDist = 0 * okapi::meter;
    bool fullAccel{true};

    public:
    SCurveMotionProfile(ProfileConstraint iConstraint);
    SCurveMotionProfile() = default;
    ~SCurveMotionProfile() = default;

    void setDistance(okapi::QLength iDistance) override;
    void setConstraint(ProfileConstraint iConstraint) override;

    okapi::QTime getTotalTime() const override;
    okapi::QLength getPosition(okapi::QTime time) const override;
    okapi::QSpeed getVelocity(okapi::QTime time) const override;
    okapi::QAcceleration getAcceleration(okapi::QTime time) const override;
    TrajectoryPoint get(okapi::QTime time) const override;
};

}