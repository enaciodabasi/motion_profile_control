/**
 * @file triangular_profile.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef TRIANGULAR_PROFILE_HPP_
#define TRIANGULAR_PROFILE_HPP_

#include "motion_profile_generators/motion_control_defs.hpp"
#include "motion_profile_generators/mpg_utils.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <optional>
#include <iostream>

namespace motion_profile_generators
{

    namespace triangular_profile
    {
        template <typename DurationType, typename ElapsedTimeType = std::ratio<1>> 
        struct TriangularProfileTimes : public ProfileTimes<DurationType, ElapsedTimeType>
        {   
            public:

            TriangularProfileTimes();
            TriangularProfileTimes(
                const std::chrono::duration<DurationType, ElapsedTimeType>& accel_time,
                const std::chrono::duration<DurationType, ElapsedTimeType>& decel_time
            );

            TriangularProfileTimes(
                TriangularProfileTimes<DurationType, ElapsedTimeType>&& other
            )
            {

                
                this->accelerationTime = other.accelerationTime;
                this->decelerationTime = other.decelerationTime;
                this->totalTime = other.totalTime;
            }

            const DurationType getAccelerationDuration() const
            {
                return accelerationTime.count();
            } 

            const DurationType getDecelerationTime() const
            {
                return decelerationTime.count();
            }

            private:

            std::chrono::duration<DurationType, ElapsedTimeType> accelerationTime;
            std::chrono::duration<DurationType, ElapsedTimeType> decelerationTime;

        };

        template <typename DurationType, typename ElapsedTimeType>
        TriangularProfileTimes<DurationType, ElapsedTimeType>::TriangularProfileTimes()
        {   
            
            this->totalTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            accelerationTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            decelerationTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            
        }

        template <typename DurationType, typename ElapsedTimeType>
        TriangularProfileTimes<DurationType, ElapsedTimeType>::TriangularProfileTimes(
            const std::chrono::duration<DurationType, ElapsedTimeType>& accel_time,
            const std::chrono::duration<DurationType, ElapsedTimeType>& decel_time
        )
        {
            accelerationTime = accel_time;
            decelerationTime = decel_time;
            this->totalTime = accelerationTime + decelerationTime;
        }

        /**
         * @brief 
         * 
         * @tparam DurationType type for time measurements, usually double or int64 
         * @tparam ControlValueType 
         * @param target_value Value to reach within the calculated profile duration.
         * @param maximum_velocity 
         * @param maximum_acceleration 
         * @return TriangularProfileTimes<DurationType> 
         */
        template<typename ControlValueType, typename DurationType, typename ElapsedTimeType = std::ratio<1>>
        TriangularProfileTimes<DurationType, ElapsedTimeType> calculateTriangularOperationTimes(
            ControlValueType target_value,
            MotionConstraints<ControlValueType>& motion_constraints
        )
        {
            
            //maximum_velocity = std::sqrt(
            //    std::abs(target_value) / maximum_acceleration
            //s);
            
            /* motion_constraints.max_increment = std::sqrt(std::abs(target_value) * motion_constraints.acceleration);
            auto timeToReachMaxVel = motion_constraints.max_increment / motion_constraints.acceleration; */
            // 1/2 * a * t * t = d --> (d * 2) / a = t^2 --> sqrt(d*2/a) = t;
            auto timeToReachMaxVel = std::sqrt(std::abs(target_value) / motion_constraints.acceleration);

            std::chrono::duration<DurationType, ElapsedTimeType>  accel_time = std::chrono::duration<DurationType, ElapsedTimeType>(timeToReachMaxVel);
            std::chrono::duration<DurationType, ElapsedTimeType>  decel_time = accel_time;
            motion_constraints.max_increment = motion_constraints.acceleration * (timeToReachMaxVel / (DurationType)2.0);

            return TriangularProfileTimes<DurationType>(accel_time, decel_time);

        }

        template<typename ControlValueType, typename DurationType, typename ElapsedTimeType = std::ratio<1>>
        TriangularProfileTimes<DurationType, ElapsedTimeType> calculateTriangularOperationTimes(
            const ControlValueType target_value,
            const DurationType target_duration,
            MotionConstraints<ControlValueType>& motion_constraints
        )
        {
            DurationType ta, td = target_duration / (DurationType)2.0;
            motion_constraints.max_increment  = target_value / ta;

            std::chrono::duration<DurationType, ElapsedTimeType> accel_time(ta);
            std::chrono::duration<DurationType, ElapsedTimeType> decel_time(td);

            return TriangularProfileTimes<DurationType>(accel_time, decel_time);
        }
        

        /**
         * @brief 
         * 
         * @tparam ReferenceType 
         * @tparam DurationType 
         * @tparam ClockType 
         * @tparam ElapsedTimeType 
         * @param previous_references 
         * @param motion_constraints 
         * @param times 
         * @param previous_timepoint 
         * @return MotionProfileReference<ReferenceType> 
         */
        template<typename ReferenceType, typename DurationType, class ClockType = std::chrono::high_resolution_clock, class ElapsedTimeType = std::ratio<1>>
        std::optional<MotionProfileReference<ReferenceType>> generateReference(
            MotionProfileReference<ReferenceType> & previous_references,
            const ReferenceType& target,
            const MotionConstraints<ReferenceType>& motion_constraints,
            const TriangularProfileTimes<DurationType>& times,
            std::chrono::time_point<ClockType>& previous_timepoint,
            const std::chrono::time_point<ClockType>& profile_start_time
        )
        {
            auto profileStartTimeCp = profile_start_time;
            std::chrono::duration<DurationType, ElapsedTimeType> elapsedTimeDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(previous_timepoint);
            std::chrono::duration<DurationType> timeElapsedSinceProfileStartedDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(profileStartTimeCp);
        
            const double elapsedTime = static_cast<DurationType>(elapsedTimeDur.count());
            /* std::cout << "Elapsed time: " << elapsedTime << std::endl; */
            const DurationType timeElapsedSinceProfileStarted = timeElapsedSinceProfileStartedDur.count();
            
            if(timeElapsedSinceProfileStarted >= times.getTotalTime()){
                std::cout << "Elapsed time " << timeElapsedSinceProfileStarted << " is larget than the target profile time " << times.getTotalTime() << std::endl;
                return std::nullopt;
            }
            MotionProfileReference<ReferenceType> newReference;
            /* if(timeElapsedSinceProfileStarted <= times.getAccelerationDuration()){
                newReference.velocity = previous_references.velocity + (motion_constraints.acceleration * elapsedTime);
                newReference.position = 0.5 * (motion_constraints.acceleration * std::pow(timeElapsedSinceProfileStarted , 2));

            }
            else if(timeElapsedSinceProfileStarted > times.getDecelerationTime()){
                newReference.velocity = previous_references.velocity - (motion_constraints.acceleration * elapsedTime);
                newReference.position = (0.5 * motion_constraints.acceleration * std::pow(timeElapsedSinceProfileStarted, 2));
            } */
            const auto directionMultiplier = sign_fnc(target);
            if(timeElapsedSinceProfileStarted <= times.getAccelerationDuration()){
                newReference.velocity = std::abs(previous_references.velocity) + (motion_constraints.acceleration * elapsedTime);
                newReference.position = 0.5 * (motion_constraints.acceleration * std::pow(timeElapsedSinceProfileStarted , 2));

            }
            else if(timeElapsedSinceProfileStarted > times.getDecelerationTime()){
                newReference.velocity = std::abs(previous_references.velocity) - (motion_constraints.acceleration * elapsedTime);
                newReference.position = (0.5 * motion_constraints.acceleration * std::pow(timeElapsedSinceProfileStarted, 2));
            }

            return newReference * (directionMultiplier);

        }
        

        /**
         * @brief 
         * 
         * @tparam ReferenceType 
         * @tparam DurationType 
         * @tparam ClockType 
         * @tparam ElapsedTimeType 
         * @param previous_references 
         * @param motion_constraints 
         * @param times 
         * @param previous_timepoint 
         * @return MotionProfileReference<ReferenceType> 
         */
        template<typename ReferenceType, typename DurationType, class ClockType = std::chrono::system_clock, class ElapsedTimeType>
        std::optional<MotionProfileReference<ReferenceType>> generateTriangularProfileReference(
            MotionProfileReference<ReferenceType> & previous_references,
            const MotionConstraints<ReferenceType>& motion_constraints,
            const TriangularProfileTimes<DurationType>& times,
            std::chrono::time_point<ClockType>& previous_timepoint,
            const std::chrono::time_point<ClockType>& profile_start_time
        );
        
    
    } // End of namespace triangular_profile

} // End of namespace motion_control_generators
#endif // TRIANGULAR_MOVE_HPP_