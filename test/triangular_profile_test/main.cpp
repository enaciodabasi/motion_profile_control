/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-15
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

#define _PLOT_DATA

#ifdef _PLOT_DATA
constexpr bool plot_data = true;
#else 
constexpr bool plot_data = false;
#endif

/* template<typename T>
bool inRange(T current, T target, T tolerance)
{
    return ((current + tolerance >= target || current - tolerance <= target) ? true : false);
} */
using namespace motion_profile_generators::triangular_profile;

#ifdef _PLOT_DATA
using PlottingData = std::unordered_map<std::string, std::vector<double>>; 
#endif

TEST(TriangularMotionProfileTest, FloatingPointReferenceTest)
{

    std::unique_ptr<PlottingData> dataToPlot;

    if constexpr(plot_data)
    {
        dataToPlot = std::make_unique<PlottingData>();
        (*dataToPlot)["velocities"] = std::vector<double>();
        (*dataToPlot)["positions"] = std::vector<double>();
        (*dataToPlot)["timestamps"] = std::vector<double>();
    }

    const double target = 1.0; // 1.0 meters
    double maxVel = 0.5;
    const double maxAcc = 0.25;
    

    TriangularProfileTimes<double> times = calculateTriangularOperationTimes<double, double>(target, maxVel, maxAcc);
    
    ASSERT_DOUBLE_EQ(times.getAccelerationDuration(), 1.4142135623730951);
    ASSERT_DOUBLE_EQ(times.getDecelerationTime(), 1.4142135623730951);
    ASSERT_DOUBLE_EQ(times.getTotalTime(), times.getAccelerationDuration() + times.getDecelerationTime());

    double distanceTraveledViaReferences = 0.0;
    const auto profileStartTime = std::chrono::high_resolution_clock::now();
    auto updateTimePoint = profileStartTime;
    MotionProfileReference<double> ref = {};
    using namespace std::chrono_literals;
    std::cout <<"Acceleration and deceleration times: " << times.getAccelerationDuration() << " " << times.getDecelerationTime() << " Total profile time: " << times.getTotalTime() << std::endl;
    while(inRange(distanceTraveledViaReferences, target, 0.0001) || std::chrono::duration_cast<std::chrono::duration<double>>((std::chrono::high_resolution_clock::now() - profileStartTime)).count() >= times.getTotalTime())
    {
        
        auto refs = generateReference<double, double, std::chrono::high_resolution_clock>(
            ref,
            target,
            (MotionConstraints<double>){maxVel, maxAcc, -maxAcc, 0.0},
            times,
            updateTimePoint,
            profileStartTime
        );
        
        if(refs)
        {   
            
            ref = refs.value();
            distanceTraveledViaReferences = refs.value().position;
            if constexpr (plot_data)
            {
                dataToPlot->at("positions").push_back(distanceTraveledViaReferences);
                dataToPlot->at("velocities").push_back(refs.value().velocity);
                auto timeElapsed = std::chrono::duration_cast<std::chrono::duration<double>>((std::chrono::high_resolution_clock::now() - profileStartTime));
                dataToPlot->at("timestamps").push_back(timeElapsed.count());
            }
/*             std::cout << distanceTraveledViaReferences << std::endl;
 */                 
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
    EXPECT_EQ(inRange(distanceTraveledViaReferences, target, 0.05), true);

}

int main(int argc, char **argv)
{

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}