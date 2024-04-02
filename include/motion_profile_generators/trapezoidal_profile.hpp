/**
 * @file trapezoidal_profile.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef TRAPEZOIDAL_PROFILE_HPP_
#define TRAPEZOIDAL_PROFILE_HPP_

#include <chrono>
#include <optional>
#include <iostream>

#include "motion_profile_generators/motion_control_defs.hpp"

namespace motion_profile_generators
{
    namespace trapezoidal_profile
    {

        template <typename DurationType = double, typename ElapsedTimeType = std::ratio<1>>
        struct TrapezoidalProfileTimes : public ProfileTimes<DurationType, ElapsedTimeType>
        {
        public:

            TrapezoidalProfileTimes();
            
            /* TrapezoidalProfileTimes(
                const DurationType accel_time,
                const DurationType constant_vel_time,
                const DurationType decel_time
            ); */

            TrapezoidalProfileTimes(
                const std::chrono::duration<DurationType, ElapsedTimeType>& accel_time,
                const std::chrono::duration<DurationType, ElapsedTimeType>& constant_vel_time,
                const std::chrono::duration<DurationType, ElapsedTimeType>& decel_time
            );

            TrapezoidalProfileTimes(TrapezoidalProfileTimes<DurationType, ElapsedTimeType>& other)
            {
                this->accelerationTime = other.accelerationTime;
                this->constantVelocityTime = other.constantVelocityTime;
                this->decelerationTime = other.decelerationTime;
                this->totalTime = other.totalTime;
            }

            TrapezoidalProfileTimes(TrapezoidalProfileTimes<DurationType, ElapsedTimeType>&& other)
            {
                this->accelerationTime = std::move(other.accelerationTime);
                this->constantVelocityTime = std::move(other.constantVelocityTime);
                this->decelerationTime = std::move(other.decelerationTime);
                this->totalTime = std::move(other.totalTime);
            }

            const DurationType getAccelerationDuration() const
            {
                return accelerationTime.count();
            } 

            const DurationType getConstantVelocityDuration() const
            {
                return constantVelocityTime.count();
            }

            const DurationType getDecelerationTime() const
            {
                return decelerationTime.count();
            }

        private:

            std::chrono::duration<DurationType, ElapsedTimeType> accelerationTime;
            std::chrono::duration<DurationType, ElapsedTimeType> decelerationTime;
            std::chrono::duration<DurationType, ElapsedTimeType> constantVelocityTime;
        };

        template <typename DurationType, typename ElapsedTimeType>
        TrapezoidalProfileTimes<DurationType, ElapsedTimeType>::TrapezoidalProfileTimes()
        {   
            
            this->totalTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            accelerationTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            decelerationTime = std::chrono::duration<DurationType, ElapsedTimeType>(0.0);
            
        }

        /* template<typename DurationType, typename ElapsedTimeType>
        TrapezoidalProfileTimes<DurationType, ElapsedTimeType>::TrapezoidalProfileTimes(
                const DurationType accel_time,
                const DurationType constant_vel_time,
                const DurationType decel_time
        )
        {
            using Duration = std::chrono::duration<DurationType, ElapsedTimeType>
            accelerationTime = Duration(accel_time);
            constantVelocityTime = Duration(constant_vel_time);
            decelerationTime = Duration(decel_time);
            this->totalTime = accelerationTime + decelerationTime + constantVelocityTime;
        } */

        template <typename DurationType, typename ElapsedTimeType>
        TrapezoidalProfileTimes<DurationType, ElapsedTimeType>::TrapezoidalProfileTimes(
            const std::chrono::duration<DurationType, ElapsedTimeType>& accel_time,
            const std::chrono::duration<DurationType, ElapsedTimeType>& constant_vel_time,
            const std::chrono::duration<DurationType, ElapsedTimeType>& decel_time
        )
        {
            accelerationTime = accel_time;
            decelerationTime = decel_time;
            constantVelocityTime = constant_vel_time;
            this->totalTime = accelerationTime + decelerationTime + constantVelocityTime;
        }

        /**
         * @brief 
         * 
         * @tparam ReferenceType 
         * @tparam DurationType 
         * @tparam ElapsedTimeType 
         * @param target_value 
         * @param maximum_velocity 
         * @param maximum_acceleration 
         * @return std::optional<TrapezoidalProfileTimes<ReferenceType, DurationType>> 
         */
        template<typename ReferenceType, typename DurationType, class ElapsedTimeType = std::ratio<1>>
        std::optional<TrapezoidalProfileTimes<DurationType, ElapsedTimeType>> calculateTrapeozidalOperationsTimes(
            const ReferenceType target_value,
            const MotionConstraints<ReferenceType>& motion_constraints
        )
        {
            // Calculate required times for acceleration and deceleration.
            std::chrono::duration<DurationType, ElapsedTimeType> tAccel = std::chrono::duration<DurationType, ElapsedTimeType>(motion_constraints.max_increment / motion_constraints.acceleration);
            std::chrono::duration<DurationType, ElapsedTimeType> tDecel = std::chrono::duration<DurationType, ElapsedTimeType>(motion_constraints.max_increment / std::abs(motion_constraints.deacceleration));

            // Calculate displacements:
            ReferenceType dAcc = 0.5*motion_constraints.max_increment*tAccel.count();
/*             std::cout <<"Max increment: " << motion_constraints.max_increment << std::endl;
            std::cout << "acc: " << motion_constraints.acceleration << std::endl; */
            ReferenceType dDec = 0.5*motion_constraints.max_increment*tDecel.count();
            /* std::cout << tAccel.count() << " " << tDecel.count() << std::endl; */
            ReferenceType dConstVel = std::abs(target_value) - (dAcc + dDec); 
            /* std::cout << "Const Vel distance: " << dConstVel << std::endl; */
            if(dConstVel <= 0)
            {
                return std::nullopt;
            }

            std::chrono::duration<DurationType, ElapsedTimeType> tConstVel = std::chrono::duration<DurationType, ElapsedTimeType>(dConstVel / motion_constraints.max_increment);
            /* std::cout << tConstVel.count() << std::endl; */
            TrapezoidalProfileTimes<DurationType, ElapsedTimeType> times(tAccel, tConstVel, tDecel);
            return times;

        }   

        template<typename ReferenceType, typename DurationType, class ElapsedTimeType = std::ratio<1>>
        std::optional<MotionProfileReference<ReferenceType>> calculateTrapeozidalOperationsTimes(
            const ReferenceType target_value,
            const DurationType target_duration,
            MotionConstraints<ReferenceType>& motion_constraints
        )
        {
            //ReferenceType avgVelocity = target_value / (DurationType)target_duration;
            /*
                Calculating profile with the Third Rule:
                Ta, Tc, Td = Tt / 3
            */
            DurationType ta, tc, td = target_duration / (DurationType)3.0;
            ReferenceType newMaxVelocity = target_value / (ta + tc);
            ReferenceType newMaxAccel = newMaxVelocity / ta;

            if(newMaxVelocity >= motion_constraints.max_increment || newMaxAccel >= motion_constraints.acceleration || (-1.0 * newMaxAccel) <= motion_constraints.deacceleration)
            {

            }
            
            ReferenceType dAcc, dDec = ReferenceType(0.5 * newMaxVelocity * ta);

            if(target_value <= dAcc + dDec)
            {
                return std::nullopt;
            }

            TrapezoidalProfileTimes<DurationType, ElapsedTimeType> times(ta, tc, td);
            return times;
        }   

        /**
         * @brief 
         * 
         * @tparam ReferenceType 
         * @tparam DurationType 
         * @tparam ClockType 
         * @tparam ElapsedTimeType 
         * @param previous_reference 
         * @param target 
         * @param motion_constraints 
         * @param times 
         * @param previous_timepoint 
         * @param profile_start_time 
         * @return std::optional<MotionProfileReference<ReferenceType>> 
         */
        template<typename ReferenceType, typename DurationType, class ClockType = std::chrono::high_resolution_clock, class ElapsedTimeType = std::ratio<1>>
        std::optional<MotionProfileReference<ReferenceType>> generateReference(
            MotionProfileReference<ReferenceType>& previous_reference,
            const ReferenceType& target,
            const MotionConstraints<ReferenceType>& motion_constraints,
            const TrapezoidalProfileTimes<DurationType, ElapsedTimeType>& times,
            std::chrono::time_point<ClockType>& previous_timepoint,
            const std::chrono::time_point<ClockType>& profile_start_time
        )
        {
        
            auto profileStartTimeCp = profile_start_time;
            std::chrono::duration<DurationType, ElapsedTimeType> elapsedTimeDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(previous_timepoint);
            std::chrono::duration<DurationType> timeElapsedSinceProfileStartedDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(profileStartTimeCp);
            
            const double elapsedTime = static_cast<DurationType>(elapsedTimeDur.count());
            
            const DurationType timeElapsedSinceProfileStarted = timeElapsedSinceProfileStartedDur.count();

            if(timeElapsedSinceProfileStarted >= times.getTotalTime()){
                std::cout << "Elapsed time " << timeElapsedSinceProfileStarted << " is larget than the target profile time " << times.getTotalTime() << std::endl;
                return std::nullopt;
            }

            MotionProfileReference<ReferenceType> newReference;
            static double dAcc, dConstVel = 0.0;
            const auto directionMultiplier = sign_fnc(target);
            if(timeElapsedSinceProfileStarted <= times.getAccelerationDuration())
            {
                newReference.position = (0.5 * motion_constraints.acceleration * (timeElapsedSinceProfileStarted * timeElapsedSinceProfileStarted));
                newReference.velocity = std::abs(previous_reference.velocity) + (elapsedTime * motion_constraints.acceleration);
                newReference.acceleration = motion_constraints.acceleration;
            }
            else if(timeElapsedSinceProfileStarted < (times.getAccelerationDuration() + times.getConstantVelocityDuration()) && timeElapsedSinceProfileStarted > times.getAccelerationDuration())
            {
                newReference.position = std::abs(previous_reference.position) + (motion_constraints.max_increment * elapsedTime);
                newReference.velocity = motion_constraints.max_increment;
                newReference.acceleration = (ReferenceType)0.0;
                dConstVel = newReference.position;
            }
            else if(timeElapsedSinceProfileStarted >= times.getAccelerationDuration() + times.getConstantVelocityDuration())
            {
                double decDur = timeElapsedSinceProfileStarted - (times.getAccelerationDuration() + times.getConstantVelocityDuration());
                newReference.position = dConstVel + (decDur * motion_constraints.max_increment) - (0.5 * motion_constraints.acceleration * (decDur * decDur));
                newReference.velocity = std::abs(previous_reference.velocity) - (std::abs(motion_constraints.deacceleration) * elapsedTime);
                newReference.acceleration = -motion_constraints.acceleration;
           } 

            return newReference * directionMultiplier;
            
        }

    } // End of namespace trapezoidal_profile
} // Endo of namespace motion_profile_generators

#endif // TRAPEZOIDAL_PROFILE_HPP_