#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "motion_profile_generators/motion_profile_controller.hpp"
#include "motion_profile_generators/motion_control_defs.hpp"
#include "motion_profile_generators/triangular_profile.hpp"
#include "motion_profile_generators/trapezoidal_profile.hpp"
#include "motion_profile_generators/scurve_profile.hpp"

TEST(MotionProfileControllerClassTest, TimeStructsPolymorphismTest)
{
    using namespace motion_profile_generators::trapezoidal_profile;
    MotionProfileController<double, double> controller;
    controller.setMotionType(MotionProfileType::Triangular);

    double target = 1.0;
    MotionConstraints<double> constraints(0.25, 0.25, 0.25, 0.5);
    controller.setMotionConstraints(constraints);
    auto setupControllerRes = controller.setupProfile(target);

    EXPECT_EQ(setupControllerRes, true);

    auto calculatedTimes = controller.m_CurrentProfileInformation.m_CurrentProfileTimes.get();    
    auto timesAsTrapez = dynamic_cast<motion_profile_generators::triangular_profile::TriangularProfileTimes<double>*>(calculatedTimes);

    EXPECT_NE(timesAsTrapez->getAccelerationDuration(), 0.0);
/*     EXPECT_NE(timesAsTrapez->getConstantVelocityDuration(), 0.0);
 */    EXPECT_NE(timesAsTrapez->getDecelerationTime(), 0.0);

    std::cout << timesAsTrapez->getAccelerationDuration()<<std::endl;
    std::cout << timesAsTrapez->getDecelerationTime()<<std::endl;
/*     std::cout << timesAsTrapez->getConstantVelocityDuration()<<std::endl;
 */    
    while(true)
    {
        auto ref = controller.generateMotionProfileReference(
        target
        );

        if(!ref)
        {
            std::cout << "Reference is nullopt\n";
            break;
        }
        std::cout << ref->velocity << std::endl;
    }
    

}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}