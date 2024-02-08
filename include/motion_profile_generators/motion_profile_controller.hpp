/**
 * @file motion_profile_controller.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MOTION_PROFILE_CONTROLLER_HPP_
#define MOTION_PROFILE_CONTROLLER_HPP_

#include <iostream>
#include <memory>
#include <optional>

#include "motion_profile_generators/motion_control_defs.hpp"
#include "motion_profile_generators/triangular_profile.hpp"
#include "motion_profile_generators/trapezoidal_profile.hpp"
#include "motion_profile_generators/scurve_profile.hpp"

template <typename ReferenceType, typename DurationType>
class MotionProfileController
{
    

public:
    MotionProfileController()
    {
        
    }

    ~MotionProfileController()
    {
    }

    
    bool setMotionType(const MotionProfileType type, bool force = false)
    {
        if ((!force && !m_IsProfileActive) || force)
        {
            m_CurrentMotionProfileType = type;
            return true;
        }
        
        return false;
    }

    template<typename ElapsedTimeType>
    std::unique_ptr<ProfileTimes<ReferenceType, DurationType>> setupProfileGeneration(const ReferenceType target, const MotionProfileType type)
    {
        

        switch (m_CurrentMotionProfileType)
        {
        case MotionProfileType::Triangular:
        {
            using namespace motion_profile_generators::triangular_profile;
            TriangularProfileTimes times = calculateTriangularOperationTimes<DurationType, ReferenceType, ElapsedTimeType>(
                target,
                m_CurrentMotionConstraints.increment,
                m_CurrentMotionConstraints.acceleration
            );
                                  
            return std::unique_ptr<TriangularProfileTimes<ReferenceType, DurationType>>(new TriangularProfileTimes<ReferenceType, DurationType>(std::move(times)));
        }
        case MotionProfileType::Trapezoidal:
        {
            
            using namespace motion_profile_generators::trapezoidal_profile;

            auto times = calculateTrapeozidalOperationsTimes(
                target,
                m_CurrentMotionConstraints
            );

            if(!times)
            {
                
                using namespace motion_profile_generators::triangular_profile;
                auto newProfileTimes = calculateTriangularOperationTimes(
                    target,
                    m_CurrentMotionConstraints
                );

                m_CurrentMotionProfileType = MotionProfileType::Triangular;

                return std::unique_ptr<TriangularProfileTimes<ReferenceType, DurationType>>(new TriangularProfileTimes<ReferenceType, DurationType>(std::move(newProfileTimes)));
            }

            return std::unique_ptr<TrapezoidalProfileTimes<ReferenceType, DurationType>>(new TrapezoidalProfileTimes<ReferenceType, DurationType>(std::move(times)));
            /* break; */
        }
        case MotionProfileType::SCurve:
        {
            using namespace motion_profile_generators::scurve_profile;

            return std::unique_ptr<SCurveProfileTimes<DurationType>>(new SCurveProfileTimes<ReferenceType, DurationType>(
                calculateSCurveOperationTimes<DurationType, ReferenceType>(target, m_CurrentMotionConstraints)
            ));
        }

        default:
            break;
        }
        

        return nullptr;
    }

    std::optional<MotionProfileReference<ReferenceType>> generateMotionProfileReference(const ReferenceType target)
    {   
        
        std::optional<MotionProfileReference<ReferenceType>> newRef;        

        switch (m_CurrentMotionProfileType)
        {
        case MotionProfileType::Triangular:
        {
            
            
            break;
        }
        case MotionProfileType::Trapezoidal:
        {

            break;
        }

        case MotionProfileType::SCurve:
        {

        }

        default:
            newRef = std::nullopt;
            break;
        }

        return newRef;
    }

private:
    MotionProfileType m_CurrentMotionProfileType;

    std::unique_ptr<ProfileTimes<ReferenceType, DurationType>> m_CurrentProfileTimes;

    MotionConstraints<ReferenceType> m_CurrentMotionConstraints;

    bool m_IsProfileActive = false;

    void reset();

};

#endif // MOTION_PROFILE_CONTROLLER_HPP_