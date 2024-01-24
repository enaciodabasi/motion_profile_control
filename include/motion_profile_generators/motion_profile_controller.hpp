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
        bool isMotionProfileSuitable = areConstraintsSuitableForCurrentMotionProfile(type);

        if(!isMotionProfileSuitable){
            // If current constraints are not suited for the desired motion profile type, change the constraint or the motion profile type:
            m_CurrentMotionProfileType = type;
        }    

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
                                  
            return std::unique_ptr<TriangularProfileTimes<ReferenceType, DurationType>>(new TriangularProfileTimes<ReferenceType, DurationType>(times));
        }
        case MotionProfileType::Trapezoidal:
        {
            
            break;
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

    bool areConstraintsSuitableForCurrentMotionProfile(const ReferenceType target, MotionProfileType& type)
    {
        
        bool profileOk = true;

        switch (type)
        {
        case MotionProfileType::Triangular:
        {
                     
            break;
        }
        case MotionProfileType::Trapezoidal:
        {
            profileOk = trapezoidalProfileCheck(target);
            // Update Constraints:
            if(!profileOk){
                type = MotionProfileType::Triangular;
            }
            break;
        }

        default:
            profileOk = false;
            break;
        }

        return profileOk;
    }

    bool trapezoidalProfileCheck(const ReferenceType target)
    {   

        ReferenceType tMaxVel = m_CurrentMotionConstraints.increment / m_CurrentMotionConstraints.acceleration;

        return (
            (fabs(target)) -
            (2.0 * fabs(0.5 * m_CurrentMotionConstraints.acceleration * (tMaxVel * tMaxVel)))
        );
    }
};

#endif // MOTION_PROFILE_CONTROLLER_HPP_