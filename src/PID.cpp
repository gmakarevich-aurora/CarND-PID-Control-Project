#include "PID.h"

#include <limits>
#include <math.h>

#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

constexpr unsigned long kTwiddleSteps = 2000;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    param_tuners.emplace_back(&this->Kp);
    param_tuners.emplace_back(&this->Kd);
    param_tuners.emplace_back(&this->Ki);

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    total_error = 0.0;
    best_error = std::numeric_limits<double>::max();
    step = 0;
    steps_since_last_twiddle = 0;
    tuner_idx = 0;
}

void PID::UpdateError(double cte) {
    if (step != 0) {
        d_error = cte - p_error;
    }
    p_error = cte;
    i_error += cte;

    total_error += pow(cte, 2);

    /**********************************************************************
     * Used to tune up the hyper parameters.
    if (steps_since_last_twiddle == kTwiddleSteps) {

        std::cout << "Tuning up the parameter: " << tuner_idx << std::endl;

        auto& param_tuner = param_tuners[tuner_idx];
        if (param_tuner.tuneUp(total_error, best_error)) {
            tuner_idx = (tuner_idx + 1) % param_tuners.size();
        }
        if (total_error < best_error) {
            std::cout << "Set of parameters: Kp: " << Kp
                      << "; Kd: " << Kd << "; Ki: " << Ki << std::endl;
            best_error = total_error;
        }
        steps_since_last_twiddle = 0;
        total_error = 0;
    }
    ***********************************************************************/
    ++steps_since_last_twiddle;
    ++step;
}

double PID::TotalError() {
    return total_error;
}

bool PID::ParamTuner::tuneUp(double error, double best_error) {
    std::cout << "State: " << state
              << "; Error: " << error
              << "; Best error: "<< best_error << std::endl;

    switch (state) {
    case START:
        *koefficient += koefficient_diff;
        state = INCREASED;
        return false;
    case INCREASED:
        if (error < best_error) {
            koefficient_diff *= 1.1;
            state = START;
            return true;
        }
        *koefficient -= 2 * koefficient_diff;
        state = DECREASED;
        return false;
    case DECREASED:
        if (error < best_error) {
            koefficient_diff *= 1.1;
        } else {
            *koefficient += koefficient_diff;
            koefficient_diff *= 0.9;
        }
        state = START;
        return true;
    }
    return false;
}

