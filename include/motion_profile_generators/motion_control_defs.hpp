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
    public:
    
    ReferenceType position;
    
    ReferenceType velocity;

    ReferenceType acceleration;

    MotionProfileReference()
    {
        position = (ReferenceType)0.0;
        velocity = (ReferenceType)0.0;
        acceleration = (ReferenceType)0.0;
    }

    MotionProfileReference<ReferenceType> operator*(const ReferenceType multiplier)
    {
        this->velocity *= multiplier;
        this->position *= multiplier;
        this->acceleration *= multiplier;

        return *this;
    }

};

template<typename T>
struct MotionConstraints
{
    public:

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

   /*  MotionConstraints<T>& operator*(const T multiplier)
    {
        acceleration *= multiplier;
        deacceleration *= multiplier;
        jerk *= multiplier;
        max_increment *= multiplier;

        return *this;
    } */

    /* void operator*(const T& multiplier)
    {
        acceleration *= multiplier;
        deacceleration *= multiplier;
        jerk *= multiplier;
        max_increment *= multiplier;
    } */

    MotionConstraints<T> operator*(const T multiplier)
    {   
        auto alteredV = *this;
        alteredV.acceleration *= multiplier;
        alteredV.deacceleration *= multiplier;
        alteredV.jerk *= multiplier;
        alteredV.max_increment *= multiplier;
        
        return alteredV;
    }
};


/* using TimePoint = std::variant<std::chrono::time_point<std::chrono::high_resolution_clock>, std::chrono::time_point<std::chrono::steady_clock>>;
 */

#endif // MOTION_CONTROL_DEFS_HPP_