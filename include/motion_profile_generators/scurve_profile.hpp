/**
 * @file scurve_profile.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SCURVE_PROFILE_HPP_
#define SCURVE_PROFILE_HPP_

#include <chrono>
#include <iostream>
#include <optional>
#include <cmath>

#include "motion_profile_generators/motion_control_defs.hpp"

namespace motion_profile_generators
{

    namespace scurve_profile
    {
        template <typename T, typename DurationType = std::ratio<1>>
        struct SCurveProfileTimes : public ProfileTimes<T, DurationType>
        {

        public:
            SCurveProfileTimes();

            SCurveProfileTimes(
                T t_1,
                T t_2,
                T t_3,
                T t_4,
                T t_5,
                T t_6,
                T t_7);

            const T getT1() const
            {
                return T1.count();
            }

            const T getT2() const
            {
                return T2.count();
            }

            const T getT3() const
            {
                return T3.count();
            }

            const T getT4() const
            {
                return T4.count();
            }

            const T getT5() const
            {
                return T5.count();
            }

            const T getT6() const
            {
                return T6.count();
            }

            const T getT7() const
            {
                return T7.count();
            }

            const T gett1() const
            {
                return t1;
            }

            const T gett2() const
            {
                return t2;
            }

            const T gett3() const
            {
                return t3;
            }

            const T gett4() const
            {
                return t4;
            }

            const T gett5() const
            {
                return t5;
            }

            const T gett6() const
            {
                return t6;
            }

            const T gett7() const
            {
                return t7;
            }

        private:
            std::chrono::duration<T, DurationType> T1;
            std::chrono::duration<T, DurationType> T2;
            std::chrono::duration<T, DurationType> T3;
            std::chrono::duration<T, DurationType> T4;
            std::chrono::duration<T, DurationType> T5;
            std::chrono::duration<T, DurationType> T6;
            std::chrono::duration<T, DurationType> T7;


            T t1;
            T t2;
            T t3;
            T t4;
            T t5;
            T t6;
            T t7;
            
        };

        template <typename T, typename DurationType>
        SCurveProfileTimes<T, DurationType>::SCurveProfileTimes()
            : ProfileTimes<T, DurationType>()
        {
        }

        template <typename T, typename DurationType>
        SCurveProfileTimes<T, DurationType>::SCurveProfileTimes(
            T t_1,
            T t_2,
            T t_3,
            T t_4,
            T t_5,
            T t_6,
            T t_7)
            : ProfileTimes<T, DurationType>()
        {
            T1 = std::chrono::duration<T, DurationType>(t_1);
            T2 = std::chrono::duration<T, DurationType>(t_2);
            T3 = std::chrono::duration<T, DurationType>(t_3);
            T4 = std::chrono::duration<T, DurationType>(t_4);
            T5 = std::chrono::duration<T, DurationType>(t_5);
            T6 = std::chrono::duration<T, DurationType>(t_6);
            T7 = std::chrono::duration<T, DurationType>(t_7);

            this->totalTime = T1 + T2 + T3 + T4 + T5 + T6 + T7;

            t1 = T1.count();
            t2 = t1 + T2.count();
            t3 = t2 + T3.count();
            t4 = t3 + T4.count();
            t5 = t4 + T5.count();
            t6 = t5 + T6.count();
            t7 = t6 + T7.count();

        }

        template <typename DurationType, typename ControlValueType, typename ElapsedTimeType = std::ratio<1>>
        SCurveProfileTimes<DurationType> calculateSCurveOperationTimes(
            ControlValueType target_value,
            const MotionConstraints<ControlValueType> &motion_constraints)
        {

            DurationType tj = motion_constraints.acceleration / motion_constraints.jerk;
            DurationType ta = (motion_constraints.max_increment / motion_constraints.acceleration) - tj;

            if(ta < 0)
            {
                tj = sqrt(motion_constraints.max_increment / motion_constraints.jerk);
                ta = 0;
            }

/*             DurationType tv = target_value / (motion_constraints.jerk * tj * ((tj + ta) - ((2*tj*ta) / 2.0)));
 */            
            DurationType tv = target_value / (motion_constraints.jerk * tj * (tj + ta)) - (2.0) * (2.0 * tj + ta) / 2.0;

            if(tv < 0)
            {
                tv = 0;
                auto c = 9 *tj*tj  - 4 * (2 * tj*tj - 2 * target_value / motion_constraints.jerk / tj / (1.0 + 1.0));

                if(c >= 0)
                {
                    ta = std::max<double>(
                        (-(3.0*tj + std::sqrt(c)) / 2.0),
                        (-(3.0 * tj - std::sqrt(c)) / 2.0) 
                    );
                }

                if(ta < 0)
                {
                    ta = 0;
                    tj = std::pow((target_value / motion_constraints.jerk / (1.0 + 1.0)), (1.0 / 3.0));
                }
            }
            std::cout << "Ta: " << ta << "Tj: " << tj << "Tv: " << tv << std::endl;
            DurationType t1 = tj;
            DurationType t2 = ta; 
            DurationType t3 = tj; 
            DurationType t4 = tv; 
            DurationType t5 = tj; 
            DurationType t6 = ta; 
            DurationType t7 = tj;

            return SCurveProfileTimes<DurationType, ElapsedTimeType>(
                t1, t2, t3, t4, t5, t6, t7);
        }

        template <typename ReferenceType, typename DurationType, class ClockType = std::chrono::high_resolution_clock, class ElapsedTimeType = std::ratio<1>>
        std::optional<MotionProfileReference<ReferenceType>> generateReference(
            MotionProfileReference<ReferenceType> &previous_references,
            const ReferenceType &target,
            const MotionConstraints<ReferenceType> &motion_constraints,
            const SCurveProfileTimes<DurationType> &times,
            std::chrono::time_point<ClockType> &previous_timepoint,
            const std::chrono::time_point<ClockType> &profile_start_time)
        {
            auto profileStartTimeCp = profile_start_time;

            std::chrono::duration<DurationType, ElapsedTimeType> elapsedTimeDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(previous_timepoint);
            std::chrono::duration<DurationType> timeElapsedSinceProfileStartedDur = calculateElapsedTime<DurationType, ClockType, ElapsedTimeType>(profileStartTimeCp);

            const double elapsedTime = static_cast<DurationType>(elapsedTimeDur.count());

            const DurationType timeElapsedSinceProfileStarted = timeElapsedSinceProfileStartedDur.count();

            if (timeElapsedSinceProfileStarted >= times.getTotalTime())
            {
                std::cout << "Elapsed time " << timeElapsedSinceProfileStarted << " is larget than the target profile time " << times.getTotalTime() << std::endl;
                return std::nullopt;
            }

            MotionProfileReference<ReferenceType> newReference;

            if(timeElapsedSinceProfileStarted <= times.gett1())
            {
                newReference.velocity = motion_constraints.jerk * std::pow(timeElapsedSinceProfileStarted, 2) / 2.0;
                newReference.position = motion_constraints.jerk * std::pow(timeElapsedSinceProfileStarted, 3) / 6.0; 
            }
            else if(timeElapsedSinceProfileStarted <= times.gett2())
            {
                newReference.velocity = motion_constraints.jerk * times.getT1() * (timeElapsedSinceProfileStarted - times.gett1()) + (motion_constraints.jerk / 2.0) * std::pow(times.getT1(), 2);

            }
            else if(timeElapsedSinceProfileStarted <= times.gett3())
            {
                newReference.velocity = -(motion_constraints.jerk / 2.0) * std::pow((timeElapsedSinceProfileStarted - times.gett2()), 2) + motion_constraints.jerk * times.getT1() * ((timeElapsedSinceProfileStarted - times.gett2()) + times.getT2()) + (motion_constraints.jerk / 2.0) * times.getT1() * times.getT1();
            }
            else if(timeElapsedSinceProfileStarted <= times.gett4())
            {
                newReference.velocity = motion_constraints.jerk * times.getT1() * (times.getT1() + times.getT2());
            }
            else if(timeElapsedSinceProfileStarted <= times.gett5())
            {
                newReference.velocity = -(motion_constraints.jerk / (2.0)) * std::pow((timeElapsedSinceProfileStarted - times.gett4()), 2)  + motion_constraints.jerk * times.getT1() * (times.getT1() + times.getT2());
            }
            else if(timeElapsedSinceProfileStarted <= times.gett6())
            {
                newReference.velocity = -(motion_constraints.jerk) * times.getT1() * (timeElapsedSinceProfileStarted - times.gett5())  + motion_constraints.jerk * times.getT1() * times.getT2()  + (motion_constraints.jerk / 2) * std::pow(times.getT1(), 2);
            }
            else if(timeElapsedSinceProfileStarted <= times.gett7())
            {
                newReference.velocity = (motion_constraints.jerk / (2.0))  * std::pow((timeElapsedSinceProfileStarted - times.gett6()), 2)   + (motion_constraints.jerk / 2.0) * std::pow(times.getT1(), 2) - (motion_constraints.jerk) * times.gett1() * (timeElapsedSinceProfileStarted - times.gett6());
            }
            else
            {
                newReference = MotionProfileReference<double>();
            }
            
            newReference.position = previous_references.velocity * elapsedTime;

            return newReference;
        }

    } // End of namespace scurve_profile

} // End of namespace motion_profile_generators

#endif // SCURVE_PROFILE_HPP_