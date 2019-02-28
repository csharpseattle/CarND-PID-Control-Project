# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introductions

In this project I build a PID controller to steer a car around a track in the Udacity simulator. The simulator uses a webSocket to provide the PID controller with the Cross Track Error (CTE), the car's speed, and the current steering angle of the car.  The PID controller uses these values to calculate a new steering value.

## PID Controller

The PID controller is named for its three components 'proporitional', 'differential (or derivative)', and 'integral'.
* The proportional component is used to correct proportional to the current cross track error.  For example, larger cross track errors result in larger corrections.  The proportional error is set to each update of the cars position in the function `UpdateError()` on line 106 of `PID.cpp`
* The integral component corrects residual error over the history of the car's motion.  The integral error is a sum of the previous CTE and is also updated in `UpdateError()` on line 115 of `PID.cpp`
* The differential component takes into account the rate of change in the cross track error between update calls and attempts to correct the overshooting by the proportional component.  This error is updated to the difference between the previous CTE and the current CTE on line 101 of `PID.cpp`

As the proportional control value corrects for the the current cross track error the car will often overshoot the center line of the track.  To correct this we use the differential value. This value uses the change in error to provide a dampening effect to the abrupt corrections provided by the proportional coefficient.  The cumulative historical correction of the integral component further refines this wavering motion.

## Tuning

The Twiddle algorithm was used to find the optimum constants for the P, I, and D coefficients.  The 3 coefficients are initialized to zero and are tuned by testing error rates for each parameter with 250 steps through the simulator.  The simulator is reset after each change.  After over 100,000 steps through the simulator the final coefficients found were:
* Kp = 0.4252619924
* Ki = 0.0002811013542
* Kd = 5.602194225

## Going further

I found that starting with different values of `Kp`, `Ki`, and `Kd` and the `dp` vector (line 20, `PID.cpp`) resulted in very different coefficients.  In addition, twiddling over longer distances and/or including more turns into the errors may give better results. More investigation is required.

An attempt was made to have different coefficients for different sections of the track by running the twiddle algorithm for each section.  Although initial results were promising, the amount of time needed to run over the entire track in this way was not doable in the time provided.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

