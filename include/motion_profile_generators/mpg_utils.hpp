/**
 * @file mpg_utils.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MPG_UTILS_HPP_
#define MPG_UTILS_HPP_

#include <chrono>
#include "motion_profile_generators/motion_control_defs.hpp"

template<typename T>
bool inRange(T current, T target, T tolerance)
{
    return ((current >= target - tolerance && current <= target + tolerance) ? true : false);
}

template<typename T>
int sign_fnc(const T& val)
{
    return (T(0) < val) - (val < T(0));
}

template<typename DurationType, class ClockType, class ElapsedTimeType>
std::chrono::duration<DurationType, ElapsedTimeType> calculateElapsedTime(
    std::chrono::time_point<ClockType>& previous_timepoint
)
{
    std::chrono::duration<DurationType, ElapsedTimeType> elapsedTimeDur;

    if constexpr(std::is_same_v<std::chrono::high_resolution_clock, ClockType>){
        auto currentTime = std::chrono::high_resolution_clock::now();
        elapsedTimeDur = currentTime - previous_timepoint;
        previous_timepoint = currentTime;
        return elapsedTimeDur;
    }   
    else if constexpr (std::is_same_v<std::chrono::steady_clock, ClockType>){
        auto currentTime = std::chrono::steady_clock::now();
        elapsedTimeDur = currentTime - previous_timepoint;
        previous_timepoint = currentTime;
        return elapsedTimeDur;
    }

    return std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
}


#endif // MPG_UTILS_HPP_