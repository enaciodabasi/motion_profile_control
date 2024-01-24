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

enum class MotionProfileType
{
    Triangular,
    Trapezoidal
};

template<typename T = double, typename DurationType = std::ratio<1>>
struct ProfileTimes
{

    public: 

    const T getTotalTime() const
    {
      
        return totalTime.count();
    }

    //using Duration = std::chrono::duration<T, DurationType>;
    protected:

    std::chrono::duration<T, DurationType> totalTime;
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


/* using TimePoint = std::variant<std::chrono::time_point<std::chrono::high_resolution_clock>, std::chrono::time_point<std::chrono::steady_clock>>;
 */

#endif // MOTION_CONTROL_DEFS_HPP_