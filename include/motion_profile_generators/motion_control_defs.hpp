/**
 * @file motion_control_defs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MOTION_CONTROL_DEFS_HPP_
#define MOTION_CONTROL_DEFS_HPP_

#include <variant>
#include <chrono>

enum class MotionProfileType : uint16_t
{
    Triangular = 0x0,
    Trapezoidal = 0x1,
    SCurve = 0x2
};

template<typename DurationType = double, typename ElapsedTimeType = std::ratio<1>>
struct ProfileTimes
{

    public: 

    ProfileTimes()
    {
        
    }

    /**
     * @brief Copy Constructor
     * 
     * @param other 
     */
    ProfileTimes(ProfileTimes<DurationType, ElapsedTimeType>& other)
    {
        this->totalTime = other.totalTime;
    }

    /**
     * @brief Move Constructor
     * 
     * @param other 
     */
    ProfileTimes(ProfileTimes<DurationType, ElapsedTimeType>&& other)
    {
        this->totalTime = std::move(other.totalTime);
    }

    virtual ~ProfileTimes()
    {
        
    }

    const DurationType getTotalTime() const
    {
      
        return totalTime.count();
    }

    //using Duration = std::chrono::duration<T, ElapsedTimeType>;
    protected:

    std::chrono::duration<DurationType, ElapsedTimeType> totalTime;
};

template<typename ReferenceType>
struct MotionProfileReference
{
    
    ReferenceType position;
    
    ReferenceType velocity;

    ReferenceType acceleration;

    MotionProfileReference()
    {
        position = (ReferenceType)0.0;
        velocity = (ReferenceType)0.0;
        acceleration = (ReferenceType)0.0;
    }

};

template<typename T>
struct MotionConstraints
{
    T max_increment;
    T acceleration;
    T deacceleration;
    T jerk;

    MotionConstraints()
    {
        max_increment = (T)0.0;
        acceleration = (T)0.0;
        deacceleration = (T)0.0;
        jerk = (T)0.0;
    }

    MotionConstraints(T inc, T acc, T decc, T jrk)
    {
        this->max_increment = inc;
        this->acceleration = acc;
        this->deacceleration = decc;
        this->jerk = jrk;
    }
};

/**
 * @brief Template struct that holds the values of initial profile information such as initial position and velocity. 
 * 
 * @tparam T 
 */
template<typename T>
struct InitialMotionInfo
{
    public:

    InitialMotionInfo()
    {

    }

    InitialMotionInfo
    (
        T initial_pos,
        T initial_vel,
        T initial_acc = (T)0.0,
        T initial_jerk = (T)0.0
    )
    {
        initialPosition = initial_pos;
        initialVelocity = initial_vel;
        initialAcceleration = initial_acc;
        initialJerk = initial_jerk;
    }

    const T getInitialPosition() const
    {
        return position;
    }

    const T getInitialVelocity() const
    {
        return velocity;
    }

    const T getInitialAcceleration() const
    {
        return acceleration;
    }

    const T getInitialJerk() const
    {
        return jerk;
    }

    private:

    T initialPosition;
    T initialVelocity;
    T initialAcceleration;
    T initialJerk;
};


/* using TimePoint = std::variant<std::chrono::time_point<std::chrono::high_resolution_clock>, std::chrono::time_point<std::chrono::steady_clock>>;
 */

#endif // MOTION_CONTROL_DEFS_HPP_