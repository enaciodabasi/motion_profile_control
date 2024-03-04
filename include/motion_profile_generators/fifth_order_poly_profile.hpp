/**
 * @file fifth_order_poly_profile.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef FIFTH_ORDER_POLY_PROFILE_HPP_
#define FIFTH_ORDER_POLY_PROFILE_HPP_

#include "motion_profile_generators/motion_control_defs.hpp"

namespace motion_profile_generators
{
    namespace fifth_order_poly_profile
    {   

        template<typename DurationType, typename ElapsedTimeType = std::ratio<1>>
        struct FifthOrderPolyProfileTimes : public ::ProfileTimes<DurationType, ElapsedTimeType>
        {
            public:

            FifthOrderPolyProfileTimes();

            FifthOrderPolyProfileTimes(
                const std::chrono::duration<DurationType, ElapsedTimeType>& total_time
            );

        };

        template<typename DurationType, typename ElapsedTimeType>
        FifthOrderPolyProfileTimes<DurationType, ElapsedTimeType>::FifthOrderPolyProfileTimes()
        {
            this->totalTime = (DurationType)0.0;
        }

        template<typename DurationType, typename ElapsedTimeType>
        FifthOrderPolyProfileTimes<DurationType, ElapsedTimeType>::FifthOrderPolyProfileTimes(
            const std::chrono::duration<DurationType, ElapsedTimeType>& total_time
        )
        {
            this->totalTime = total_time;
        }

        template<typename ReferenceType, typename DurationType, typename ElapsedTimeType>
        const FifthOrderPolyProfileTimes<DurationType, ElapsedTimeType> calculateFifthOrderPolyProfileTimes(
            const MotionConstraints<ReferenceType>& constraints,
            const InitialMotionInfo<ReferenceType>& initial_motion_info
        )
        {
            
             
        }

    } // End of namespace fifth_order_poly_profile.
} // End of namespace motion_profile_generators

#endif // FIFTH_ORDER_POLY_PROFILE_HPP_