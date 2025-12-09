#ifndef PID_AUTOTUNER_H
#define PID_AUTOTUNER_H

#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>
#include "motion_control.h"

class RelayTuner {
private:
    double targetVal;
    double outputStep; // The motor speed to use for tuning (e.g. 2.0 rad/s)
    
    // State
    bool isTuning;
    bool lastOutputPositive;
    double lastPeakTime;
    
    // Data Collection
    std::vector<double> peaks;   // Amplitudes
    std::vector<double> periods; // Cycle times

public:
    RelayTuner(double target, double step) 
        : targetVal(target), outputStep(step), 
          isTuning(false), lastPeakTime(0), lastOutputPositive(true) {}

    void start(double currentTime) {
        isTuning = true;
        lastPeakTime = currentTime;
        peaks.clear();
        periods.clear();
        lastOutputPositive = true;
        std::cout << "--- Autotuning Started ---" << std::endl;
    }

    // Call this every loop iteration. Returns the motor command.
    double runStep(double currentSensorVal, double currentTime) {
        if (!isTuning) return 0.0;

        double error = targetVal - currentSensorVal;
        double output = 0.0;

        // 1. Relay Logic (Bang-Bang Control)
        if (error > 0) output = outputStep;
        else output = -outputStep;

        // 2. Detect Zero Crossing (Cycle Switch)
        bool currentOutputPositive = (output > 0);
        
        if (currentOutputPositive != lastOutputPositive) {
            // We just crossed zero.
            // The max error observed just before crossing is roughly the amplitude.
            double cycleAmplitude = std::abs(error); 
            peaks.push_back(cycleAmplitude);

            // Calculate Half-Period
            double dt = currentTime - lastPeakTime;
            periods.push_back(dt);
            
            lastPeakTime = currentTime;
            lastOutputPositive = currentOutputPositive;
            
            // Log progress
            std::cout << "Cycle: " << peaks.size() << " | Amp: " << cycleAmplitude << std::endl;
        }

        // 3. Stop Condition (e.g., 10 half-cycles = 5 full waves)
        if (peaks.size() >= 10) { 
            isTuning = false;
            std::cout << "--- Tuning Data Collected ---" << std::endl;
        }

        return output;
    }

    bool isFinished() const {
        return !isTuning;
    }

    // Calculate and return the new PID Configuration
    PIDConfig calculatePID() {
        PIDConfig config;
        
        if (peaks.size() < 4 || periods.size() < 4) {
            std::cout << "Not enough data to tune." << std::endl;
            return config;
        }

        // A. Average Amplitude (a) - Skip first 2 messy cycles
        double avgAmp = std::accumulate(peaks.begin() + 2, peaks.end(), 0.0);
        avgAmp /= (peaks.size() - 2);

        // B. Average Full Period (Tu) - Sum of half-periods * 2
        double avgHalfPeriod = std::accumulate(periods.begin() + 2, periods.end(), 0.0);
        avgHalfPeriod /= (periods.size() - 2);
        double Tu = avgHalfPeriod * 2.0;

        // C. Ultimate Gain (Ku) formula for Relay Tuning
        double Ku = (4.0 * outputStep) / (3.14159 * avgAmp);

        std::cout << "Results -> Ku: " << Ku << " Tu: " << Tu << std::endl;

        // D. Ziegler-Nichols Rules (Classic PID)
        config.Kp = 0.6 * Ku;
        config.Ki = (2.0 * config.Kp) / Tu;
        config.Kd = (config.Kp * Tu) / 8.0;
        
        // Set reasonable limits based on tuning step
        config.max_output = outputStep * 1.5;
        config.min_output = -outputStep * 1.5;
        config.max_integral = outputStep; 

        return config;
    }
};

#endif