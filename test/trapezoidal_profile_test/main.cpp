/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <gtest/gtest.h>
#include <motion_profile_generators/motion_profile.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <unordered_map>

#include <matplot/matplot.h>

#include "motion_profile_generators/trapezoidal_profile.hpp"

#define _PLOT_DATA

#ifdef _PLOT_DATA
constexpr bool plot_data = true;
using PlottingData = std::unordered_map<std::string, std::vector<double>>; 
#else 
constexpr bool plot_data = false;
#endif

using namespace motion_profile_generators;

TEST(TrapezoidalMotionProfileTest, FloatingPointReferenceTest)
{

    std::unique_ptr<PlottingData> dataToPlot;

    if constexpr(plot_data)
    {
        dataToPlot = std::make_unique<PlottingData>();
        (*dataToPlot)["velocities"] = std::vector<double>();
        (*dataToPlot)["positions"] = std::vector<double>();
        (*dataToPlot)["timestamps"] = std::vector<double>();
    }

    const double target = 1.0;
    double max_vel = 0.25;
    const double max_acc = 0.5;

    const auto times = trapezoidal_profile::calculateTrapeozidalOperationsTimes<double, double>(target, max_vel, max_acc);

    
    std::cout << "tAcc: " << times.getAccelerationDuration() << " tConstVel: " << times.getConstantVelocityDuration() << " tDec: " << times.getDecelerationTime() << std::endl;
    
    MotionProfileReference<double> reference = {};
    double distanceTraveledViaReferences = 0.0;
    using namespace std::chrono_literals;
    const auto profileStartTime = std::chrono::high_resolution_clock::now();
    auto updateTimePoint = profileStartTime;
    MotionConstraints<double> constraints = {max_vel, max_acc, max_acc, 0.0};
    while(inRange(distanceTraveledViaReferences, target, 0.0001))
    {
        auto newRef = trapezoidal_profile::generateReference<double, double, std::chrono::high_resolution_clock, std::ratio<1>>(
            reference,
            target,
            constraints,
            times,
            updateTimePoint,
            profileStartTime
        );

        if(newRef){
            reference = newRef.value();
            distanceTraveledViaReferences = reference.position;
            std::cout << distanceTraveledViaReferences << " " << reference.velocity << std::endl;
            if constexpr (plot_data)
            {
                dataToPlot->at("positions").push_back(distanceTraveledViaReferences);
                dataToPlot->at("velocities").push_back(reference.velocity);
                auto timeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>((std::chrono::high_resolution_clock::now() - profileStartTime));
                dataToPlot->at("timestamps").push_back(timeElapsed.count());
            }
        }
        else
        {
            break;
        }

        std::this_thread::sleep_for(2ms);

    }

    std::chrono::time_point<std::chrono::high_resolution_clock> endPoint = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1>> elapsedTimeTotal = endPoint - profileStartTime;

    std::cout << "Distance traveled: " << distanceTraveledViaReferences << " \nElapsed Time: " << elapsedTimeTotal.count() << std::endl;

/*     ASSERT_NE(ref, std::nullopt);
 */ 

    if constexpr(plot_data)
    {
        matplot::plot(
            dataToPlot->at("timestamps"), dataToPlot->at("positions"), 
            dataToPlot->at("timestamps"), dataToPlot->at("velocities")
        );

        ::matplot::legend({"positions", "velocities"});
        matplot::show();
    }

}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}