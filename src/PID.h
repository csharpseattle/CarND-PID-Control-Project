#ifndef PID_H
#define PID_H

#include <vector>
#include <deque>

enum Twiddle_State
{
    twiddle_start,
    twiddled_up,
    twiddled_back,
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
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();

    /**
     * Find the optimum params
     * from the previous 100 steps
     */
    double getSpeed();


private:

    bool done;
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;


    double prev_cte;
    double curr_cte;
    double diff_cte;
    double int_cte;

    //
    // our count of the number
    // of updates we've performed
    //
    int steps;

    int pid_index;
    enum Twiddle_State pid_state;

    double best_error;
    std::vector<double> p;
    std::vector<double> dp;
    std::deque<double> ctes;
};

#endif  // PID_H
