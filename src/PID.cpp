#include "PID.h"
#include <numeric>
#include <iostream>
#include <iomanip>
#include <cmath>
PID::PID()  {}
PID::~PID() {}

const int TWIDDLE_NUMBER_STEPS = 250;
const double TWIDDLE_TOLERANCE = 0.00001;
const int INTEGRAL_ERROR_COUNT = 200;

void PID::Init(double Kp_, double Kd_, double Ki_)
{
    //
    // Initialize PID coefficients and errors
    //
    p       = {Kp_, Kd_, Ki_};
    dp      = {1.591282316e-06, 5.35810784e-06, 1.811351647e-08};
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    //
    // If twiddling we will gather errors for CTEs
    // over this many steps.
    //
    twiddle_steps = 2 * TWIDDLE_NUMBER_STEPS;

    //
    // Our step count to keep track of steps when
    // twiddling.
    //
    steps = 0;

    //
    // The index into the 'p' array (coefficients) that
    // we are currently twiddling.
    //
    pid_index = 0;

    //
    // the current state of the twiddle.
    //
    pid_state = twiddle_start;

    //
    // if the sum of the dp parameters is lower
    // than the TWIDDLE_TOLERANCE then our
    // twiddling is already done.
    //
    twiddle_completed = (std::accumulate(dp.begin(), dp.end(), 0.0) < TWIDDLE_TOLERANCE);
}


//
// Helper function to calculate the
// average error during a twiddle operation.
//
double PID::calculateAvgTwiddleError()
{
    int divideBy = 0;
    double error = 0.0;

    for (int i = 0; i < ctes.size(); ++i)
    {
        if (i >= TWIDDLE_NUMBER_STEPS)
        {
            divideBy++;
            error += ctes[i];
        }
    }

    error /= divideBy;
    return error;
}


//
// Updates the Proportional, Integral, and Differential errors.
// Returns the reset state of the twiddle algorithm.
//  - returns true if we need to reset to beginning of the track
//  - returns false if we do not.
//
bool PID::UpdateError(double cte)
{
    //
    // Increment the number of steps
    // so far.  We use this to know
    // if we have enough CTEs to twiddle
    // with
    //
    steps++;

    //
    // The differential portion of the error.  Before
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
    // integral - sum of cross track errors.  We use
    // a deque so that we can maintain only the last
    // |INTEGRAL_ERROR_COUNT| CTEs
    //
    // Push back the current CTE
    //
    i_errors.push_back(cte);
    while (i_errors.size() > INTEGRAL_ERROR_COUNT)
    {
        //
        // Pop any excess CTEs
        //
        i_errors.pop_front();
    }

    //
    // The i_error is the sum of the prevous CTEs
    //
    i_error = std::accumulate(i_errors.begin(), i_errors.end(), 0.0);

    //
    // if the twiddling is done we return false
    // to indicate that the sim does not need to
    // be reset.
    //
    if (twiddle_completed)
    {
        return false;
    }

    //
    // If we are still capturing CTEs during a twiddle step
    // we simply push back errors into our CTE vector and
    // return false to keep the sim going.  We gather a count of
    // |(2 * twiddle_steps)| CTEs before calculating the error.
    //
    if ((pid_state != twiddle_start) && (ctes.size() < (twiddle_steps)))
    {
        ctes.push_back(cte * cte);
        return false;
    }

    //
    // Once we have 2 * twiddle_steps values in our CTE
    // history we can use them to twiddle our
    // PID coefficients to find the optimum values
    //
    switch (pid_state)
    {
        case twiddle_start:
        {
            //
            // Starting a twiddle for the coefficient
            // at pid_index:  We twiddle up by the
            // value in the dp vector and return true
            // to tell the sim to reset.
            //
            p[pid_index] += dp[pid_index];
            pid_state = twiddled_up;
            return true;
        }

        case twiddled_up:
        {
            //
            // We have now run the sim for |2 * twiddle_steps|
            // and we can now calculate the error
            //
            double error = calculateAvgTwiddleError();

            if (error < best_error)
            {
                //
                // twiddling up was successful in
                // obtaining a lower error value.  move
                // the dp value up for the next time around.
                //
                best_error = error;
                dp[pid_index] *= 1.1;

                //
                // Change state to done to move to the
                // next 'p' value below.
                //
                pid_state = twiddle_done;
            }
            else
            {
                //
                // twiddling up didn't result in a lower
                // error.  Try something lower than the dp
                // value.
                //
                p[pid_index] -= 2 * dp[pid_index];
                pid_state = twiddled_back;

                //
                // let the sim know to reset and start
                // from the beginning.
                //
                return true;
            }
            break;
        }

        case twiddled_back:
        {
            //
            // We have now run the sim for |2 * twiddle_steps|
            // after trying the lower twiddle value.
            //
            double error = calculateAvgTwiddleError();

            if (error < best_error)
            {
                //
                // Found a lower error.  update dp
                // and set state to move to the next
                // PID coefficient.
                //
                best_error = error;
                dp[pid_index] *= 1.1;
                pid_state = twiddle_done;
            }
            else
            {
                //
                // we didn't find a lower error. Try a
                // lower dp value next time around.
                //
                p[pid_index] += dp[pid_index];
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
        //
        // Move to the next coefficient in the vector
        // looping around if necessary.
        //
        pid_state = twiddle_start;
        pid_index = (pid_index + 1) % 3;

        //
        // We are done twiddling when the sum of the dp
        // values is less than the TWIDDLE_TOLERANCE.
        //
        double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
        if (sum < TWIDDLE_TOLERANCE)
        {
            twiddle_completed = true;
        }

        //
        // Since we are starting a new index in the
        // p array, we tell the sim to reset to the beginning.
        //
        return true;
    }

    return false;
}


//
// This method is called by main.cpp to find
// the new steering value.
//
double PID::TotalError()
{
    //
    // Calculate and return the total error
    //
    double error = (-p[0] * p_error) + (-p[1] * d_error) + (-p[2] * i_error);

    //
    // clamp to [-1, 1]
    //
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


//
// Reset some values when the simulator resets.
// main.cpp calls this function when it
// sends the 'reset' message to the sim.
//
void PID::Reset()
{
    steps   = 0;
    i_error = 0.0;
    ctes.clear();
}


//
// This method lets main.cpp know what speed to
// message to the sim.  Currently this just returns
// .3 but could be where another PID contoller handles
// the speed values.
//
double PID::getSpeed()
{
    return 0.3;
}


