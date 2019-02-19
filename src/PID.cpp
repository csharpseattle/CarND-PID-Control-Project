#include "PID.h"
#include <numeric>
#include <iostream>

PID::PID()  {}
PID::~PID() {}

const int TWIDDLE_OVER_STEPS = 5;
const double TWIDDLE_TOLERANCE = 0.00001;

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    //
    // Initialize PID coefficients (and errors, if needed)
    //
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    p  = {Kp_, Kd_, Ki_};
    //dp = {0.00682484, .1, .001};
    p = {0.0, 0.0, 0.0};
    dp = {1.0, 1.0, 1.0};

    // p  = {0.474599, 0.00948717, 0.000500353};
    // dp = {8.91526e-14, 1.62096e-15, 1.98117e-16};


    // p = {0.363958, 0.0061051, 0.000454494};
    // dp = {0.00969212, 0.00017622, 2.1538e-05};

    // p = {0.438739, 0.00711602, 0.000617204};
    // dp = {8.84913e-05, 1.7877e-06, 2.18496e-07};

    // p = {0.439279, 0.00712823, 0.000618538};
    // dp = {9.06785e-07, 3.04058e-08, 2.48774e-09};

    // p = {0.439287, 0.00712842, 0.000618553};
    // dp = {9.10707e-09, 2.77612e-10, 1.85839e-11};

    // p = {0.439287, 0.00712842, 0.000618553};
    // dp = {9.09815e-12, 2.52127e-13, 1.68779e-14};

    steps = 0;
    best_error = std::numeric_limits<double>::max();

    pid_index = 0;
    pid_state = twiddle_start;
    done = false;
}


void PID::UpdateError(double cte)
{
    //
    // Increment the number of steps
    // so far.  We use this to know
    // if we have enough CTEs to twiddle
    // wieh
    //
    steps++;

    //
    // The derivative portion of the error.  Before
    // we set p_error with the current CTE we can
    // use as the previous crosstrack error to
    // calculate d_error
    //
    d_error = cte - p_error;

    //
    // Now we can set the current crosstrack error.
    //
    p_error = cte;

    //
    // integral - sum of cross track errors.
    //
    i_error += cte;

    if (done)
    {
        return;
    }

    if ((pid_state != twiddle_start) && (ctes.size() < TWIDDLE_OVER_STEPS))
    {
        ctes.push_back(cte * cte);
        return;
    }

    //
    // If we have at least 100 values in our CTE
    // history we can use them to twiddle our
    // PID coefficitents to find the optimum values
    //


    switch (pid_state)
    {
        case twiddle_start:
        {
            std::cout << "Start p[" << pid_index << "] += " << dp[pid_index] << " = ";
            p[pid_index] += dp[pid_index];
            std::cout << p[pid_index] << "\n";
            ctes.clear();
            i_error = 0.0;
            pid_state = twiddled_up;
            break;
        }

        case twiddled_up:
        {
            double error = 0.0;
            int divideBy = 0;
            for (int i = 0; i < ctes.size(); ++i)
            {
                divideBy++;
                error += ctes[i];
            }

            error /= divideBy;
            std::cout << "UP: Step: " << steps << " Error: " << error << " p[" << pid_index << "] = " << p[pid_index] << "\n";

            if (error < best_error)
            {
                best_error = error;
                dp[pid_index] *= 1.1;
                pid_state = twiddle_done;
            }
            else
            {
                p[pid_index] -= 2 * dp[pid_index];
                std::cout << "FLIP : Step: " << steps << " Error: " << error << " now p[" << pid_index << "] = " << p[pid_index] << "\n";
                ctes.clear();
                i_error = 0.0;
                pid_state = twiddled_back;
            }
            break;
        }

        case twiddled_back:
        {
            double error = 0.0;
            int divideBy = 0;
            for (int i = 0; i < ctes.size(); ++i)
            {
                divideBy++;
                error += ctes[i];
            }

            error /= divideBy;

            if (error < best_error)
            {
                best_error = error;
                dp[pid_index] *= 1.1;
                pid_state = twiddle_done;
            }
            else
            {
                p[pid_index] += dp[pid_index];
                std::cout << "NO GO : Step: " << steps << " Error: " << error << " now p[" << pid_index << "] = " << p[pid_index] << "\n";
                ctes.clear();
                dp[pid_index] *= 0.9;
                pid_state = twiddle_done;
            }
            break;
        }

        case twiddle_done:
        break;
    }

    if (pid_state == twiddle_done)
    {
        pid_index = (pid_index + 1) % 3;
        pid_state = twiddle_start;
        ctes.clear();
        i_error = 0.0;

        double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
        std::cout << "------------------- " << sum << " -------------------------------\n";
        if (sum < TWIDDLE_TOLERANCE)
        {
            std::cout << "--------------------------------------------------\n";
            std::cout << "--------------------------------------------------\n";
            std::cout << "p  = {" << p[0] << ", " << p[1] << ", " << p[2] << "};\n";
            std::cout << "dp = {" << dp[0] << ", " << dp[1] << ", " << dp[2] << "};\n";
            std::cout << "--------------------------------------------------\n";
            std::cout << "--------------------------------------------------\n";
            done = true;
        }
    }
}


double PID::TotalError()
{
    //
    // Calculate and return the total error
    //
    double error = (-p[0] * p_error) + (-p[1] * d_error) + (-p[2] * i_error);
    if (error < -1.0)
    {
        error = -1.0;
    }

    if (error > 1.0)
    {
        error = 1.0;
    }

    return error;

}

double PID::getSpeed()
{
    return 0.05;
}


