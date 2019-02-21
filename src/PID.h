#ifndef PID_H
#define PID_H

#include <vector>
#include <deque>

enum Twiddle_State
{
    //
    // The twiddle_start state occurs before
    // any changes to the parameters.
    //
    twiddle_start,

    //
    // The twiddled_up state occurs when we
    // have updated a coefficient (P, I, or D)
    // by multiplying by 1.1.  We reset the
    // sim and gather CTE data by driving dow
    // the track before moving to another state.
    //
    twiddled_up,

    //
    // The twiddled_back state occurs when
    // the error from twiddling up is not less
    // the the lowest error.  We reduce the
    // coefficient and gather more CTE data.
    //
    twiddled_back,

    //
    // twiddle_done occurs when we've either
    // found a better error or gone through
    // all other states. After this state we will
    // move on to the next coefficient (P, I, or D)
    // and move the state to twiddle_start.
    //
    twiddle_done
};


class PID
{
public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_);

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    bool UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();

    /**
     * Return the speed.
     * @output the speed of the car
     */
    double getSpeed();

    /**
     * Reset the twiddle parameters.  Called
     * when the simulator is reset to the
     * start state.
     * @output none.
     */
    void Reset();

private:

    bool twiddle_completed;
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;

    //
    // our count of the number
    // of updates we've performed
    //
    int steps;

    //
    // the number of steps to use
    // when gathering twiddle Cross
    // track errors.
    //
    int twiddle_steps;

    //
    // the index into the 'p' vector below
    // that we currently twiddling.
    //
    int pid_index;

    //
    // The current state of the twiddle
    // algorithm.
    //
    enum Twiddle_State pid_state;

    //
    // the best error currently found by
    // the twiddle algorithm
    //
    double best_error;

    //
    // a vector to hold the Proportional, Differential, and
    // Integral parameters.
    //
    //
    std::vector<double> p;
    std::vector<double> dp;

    //
    // Deques used for maintaining a collection of a set
    // number of error values.
    //
    std::deque<double> ctes;
    std::deque<double> i_errors;

    //
    // helper function to calculate the
    // average error during a twiddle operation.
    //
    double calculateAvgTwiddleError();
};

#endif  // PID_H
