## Model Predictive Controller (MPC) Project
### Writeup by Tony Knight - 2017/10/03

---


<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/MPC_Screenshot2.png?raw=true"  width=700>

<i><u>Figure 1: Snapshot from MPC Video</u></i>

[Link to Youtube video of car running two laps around track](https://youtu.be/4EWSR1dNmNM)

---

### Running MPC


The model can be run from the build directory by calling:
 
```sh
$./mpc
```
which will execute the model using a target velocity of 70 mph and use 14 timesteps of 0.05s.

Alternatively, you can specify a different target velocity like this:

```sh
$./mpc 85.
```

You can also specify the timestep duration and number of timesteps from the CLI as follows:

```sh
$./mpc 55 0.1 12
```

With the default timestep settings, the car can safely go around the track at speeds of up to 75 mph.  At 75 mph and beyond the car will often run up against the concrete ledge.  The car will generally crash within a couple of laps at speeds of 85 mph and beyond. 

### Kinematic Car Model

The model predictive controller utilizes a simplified Kinematic model, as opposed to a full dynamic model of the car.  It does not take into account factors like air resistance, road / tire friction, lateral forces, or gravity.  The model state consists of the car location (x,y), speed (v), and direction (Psi).  The speed and direction of the car change over time based on acceleration (a), and steering direction (delta) respectively.

The car can be controlled by two actuators: throttle and steering.  The throttle rate does not correspond exactly to an acceleration rate, as a given throttle rate in the simulator will not be able to accelerate a car beyond a speed (in mph) approximately 100 times the throttle rate.  Negative throttle rates act as reverse acceleration which is equivalent to braking.  The steering is limited to +/- 25 degrees which is input as a control of -1 to 1 to the simulator.

In our model, the following equations are used to determine the new position, speed, and direction of the car at time t which is one interval dt after the last known position.  Lf is a constant which is the distance from the front of the car to the centroid of the car mass.

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/stateupdate.png?raw=true"  width=250>



### Number and Duration of Timesteps in Model

I initially tested the use of a time interval of 0.05 s and tuned other parameters to allow the car to go around the track at speeds of 40 mph.  I found that the optimal value of N (number of timesteps) to use was 22 at this speed, resulting in a horizon of about 1.1 seconds.  I tested these parameters when switching to using a timestep of duration of 0.1s, and found I had to retune the parameters and reduce the number of timesteps to 12.  I generally found that reducing the number of timesteps below the optimal rate made the car go straighter but potentially in the wrong direction, whereas increasing the number of timesteps beyond the optimal value range increased wobbles particularly after exiting a curve.  

Any increase in computation time required in the solver routine due to an increased number of timesteps (up to 22) was not observed to be a problem affecting results on my PC.  However, in the event of being constrained by processing power, this could be a reason to use a longer timestep interval which would require less timesteps to solve.

After tuning other parameters extensively I developed the following relationship between the optimal number of timesteps and the desired velocity when using a timestep of 0.05 seconds duration (shown in figure below).

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/OptimalTimesteps.png?raw=true"  width=500>

Interestingly, as shown in the diagram below, this produced horizon lengths which increased linearly to about 25 m for speeds below 50 mph, and then remained constant near this length until the speed increased beyond 85 mph 

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/OptimalHorizonDistance.png?raw=true"  width=500>





### Solver Cost Equations

The following equations were used to calculate the cost of a specific set of actuator values at each time setting.

I initially determined the desired speed in m/s and the desired throttle actuator value corresponding to acceleration.

```C++
ref_v = desired_velocity * 1609 / 3600;
ref_a = desired_velocity / 100;
```

The costs were then calculated to minimize cross track error, difference in direction, and difference in velocity from the desired values at each time step.

```C++
// Minimize cross track error, incorrect direction, and velocity.
for (t = 0; t < N - 1; t++) 
{
  //minimize cross track error
  fg[0] += cte_factor * squared(vars[cte_start + t] - ref_cte);    

  //minimize direction error
  fg[0] += epsi_factor * squared(vars[epsi_start + t] - ref_epsi); 

  //try to maintain a target speed
  fg[0] += v_factor * squared(vars[v_start + t] - ref_v);
}
```

I added the calculations below adding costs for steering (I ended up rating the cost of steering at 0 in my solutions), and for using an acceleration rate other than the desired rate.

```C++
// Minimize the use of actuators.
for (t = 0; t < N - 1; t++) 
{
  //minimize steering angles
  fg[0] += steer_factor * squared(vars[delta_start + t] - ref_delta);

  //minimize use of accelerator/brake
  fg[0] += accel_factor * squared(vars[a_start + t] - ref_a);
}
```

I also added costs for changing the steering angle and acceleration rate between timesteps in the solver.

```C++
// Minimize the value gap between sequential actuations.
for (t = 0; t < N - 2; t++)
{
  //remove jerkiness in steering
  fg[0] += delta_factor * squared((vars[delta_start + t + 1] - vars[delta_start + t]) - ref_delta_delta);

  //remove jerkiness in acceleration/braking
  fg[0] += delta_a_factor * squared((vars[a_start + t + 1] - vars[a_start + t]) - ref_delta_a);
}
```

Finally I added in a cost for changing the cross track error between steps, which should bias towards distributing the movement towards the waypoint path over the length of the horizon. 

```C++
for (t = 1; t < N - 1; t++) 
{
  //minimize change in cte between timesteps - straighten curve
  fg[0] += delta_cte_factor * squared((vars[cte_start + t + 1] - vars[cte_start + t]) - ref_cte);
}
```

The following sets of factors applied in the equations above were experimentally determined in the simulator for time intervals of 0.05 and 0.1s:

### <b>Table 1:</b>
|Timestep Duration | 0.05s|
|:---|:--|
|cte_factor | 4,000|
|epsi_factor | 500|
|v_factor | 30|
|steer_factor | 0 |
|accel_factor | 10|
|delta_factor | 1|
|delta_a_factor | 3|
|delta_cte_factor | 750,000|


### <b>Table 2:</b>

|Timestep Duration | 0.1s|
|:---|:-|
|cte_factor | 3,750|
|epsi_factor | 2,400|
|v_factor | 10|
|steer_factor | 0 |
|accel_factor | 125|
|delta_factor | 0|
|delta_a_factor | 10|
|delta_cte_factor | 22,500|



### Waypoint Polynomial fit

I initially used a third order polynomial fit to the 6 waypoints provided using the provided polyfit function.  However, I found that occasionally the end of the polynomial line closest to the car "jogged" to one side for one or two steps.  This resulted in the car moving to the side at this point also. I attempted to resolve this "feature" by inserting an earlier control waypoint at the start of the line (storing the first waypoint between program iterations, and then inserting it as the initial waypoint).  However, fitting the curve to an additional waypoint caused problems because the longer curve became more complex.  I "fixed" this by removing the final set of waypoints so the curve only had to fit 6 waypoints again.  

During one of the tuning runs, I ran an experiment where I used a 2nd order polynomial fit instead of a 3rd order fit.  This provided better results on the track, possibly because each curve segment on the track can easily be modeled by a 2nd order polynomial, and the solver does not try to compensate for future curves in the opposite direction to the upcoming curve.  After this I kept using the 2nd order fit to achieve greater speeds on the track.  The code in the model is implemented so that it is easy to switch between fits of different orders.


### Latency

The solution I implemented to get around a known 100 mS delay in applying actuator values was to calculate the approximate pose of the car 100 mSecs into the future, based on the known location, speed, direction and steering angle of the car (and disregarding effects of acceleration or changes in steering during this interval).  This projected position is then used to calculate the relative locations of the waypoints and the cte before attempting to solve for the lowest cost fit.  

``` C++
          px = px + (v * CppAD::cos(psi) * (latency_delay/1000.0));
          py = py + (v * CppAD::sin(psi) * (latency_delay/1000.0));
          psi = psi - (v / Lf * steer / (0.436332 * Lf) * (latency_delay/1000.0));
```

A solution could also have been implemented within the model where actuator settings are not applied until after 0.1 seconds.  However, this would have relied on the latency being a multiple of the model step length to get good results.


## Additional Experiments


### Increasing costs in future

I ran a couple of tests to see if increasing the cost of actions further in the future would result in earlier responses that would enable better performance on the track.  I multiplied the cost of state and actuator errors by ```(t + tunable_constant)```, where the tunable constant was an integer between 2 and N/2, and t was the iteration number.  The initial tests did not produce observably better results, so I abandoned this approach.

### Curve Direction and Sharpness

I decided that the optimal path of the car (In order to allow it to make it around the track at higher speeds) would be to the inside of the curve produced by the waypoints.  I realized that the sign and magnitude of the second order coefficient of the polyfit line described the direction and sharpness of the curve.  I used this value to calculate reference CTE values for the solver cost calculations, and tuned these equations based on the speed of the car as follows:

```C++
   //ref_cte - adjust ref_cte so that predicted path should be biased to inside of waypoint curve
    // optimal value needed to increase with velocity (also inversely to number of timesteps) 
    if (desired_velocity > 70)
    {//Above 70 mph
      ref_cte = (-coeffs[2]+0.0013) * (620 + 30 * (desired_velocity - 70));
    }
    else if (desired_velocity > 50)
    {//between 51 and 70 mph
      ref_cte = -coeffs[2] * (300 + 16 * (desired_velocity-50));
    }
    else
    {//between 1 and 50 mph
      ref_cte = -coeffs[2] * 6 * desired_velocity;
    }
```


### Biasing Reference CTE to compensate for Critical Area of Track

The following figure shows a map of the waypoints on the track, and I have labelled the main curves and direction of travel.  


<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/Waypoints.png?raw=true"  width=500>

The critical area of the track where my implementation of the MPC controller had the most problems was in or after curve number 3, where the car either hits the concrete edge on the outside (left side) of the main curve or hits the concrete edge on the opposite (right) side of the track after coming through the curve when it exceeds a safe speed.  In order to push the maximum speed higher on this track, I globally biased the value of the second coefficient used to calculate the reference_CTE when the speed increased beyond 70 mph.  This generally results in the car moving to the right on the track.  I did not like this solution as it is only applicable to this track, and would cause the car to perform worse on other tracks, or even travelling around the same track in the opposite direction.  The figure below shows how the reference_CTE value varied according to speed and on the direction of the curve (calculated for abs(coeff[2]) = 0.2).

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/OptimalRef_CTE.png?raw=true"  width=500>


### Throttle Constraints (No Braking!)

I did not like the way that braking (reverse throttle) was applied by the solver, generally finding that it was applied too late in curves.  This might be because the more dynamic model in the simulator produced different results than the kinematic model was predicting.  I decided to limit the solver to using coasting (near zero throttle) as the only means whereby velocity could be reduced.  The throttle actuator was limited between 0.05 and 0.95, which I found produced good results.


### CTE

The generalized solution to calculate the cross track error is to take the y intercept of the polyfit waypoints curve using the "car" co-ordinates (where the car is at 0,0 and is heading along the x-axis).  This approximation becomes invalid as the difference in the directions of the polyfit line and of the car diverge.  This could occur if the car spins out along the waypoint line.

I attempted to calculate a slightly better approximation of the CTE by 
1) locating two points on the polyline close to the CTE, 
2) calculating the angle of the line at that location (in Car co-ordinates),
3) transforming the location and direction of the co-ordinates so that the polyfit line lay along the x axis near the CTE.  
The transformed y co-ordinate of the car represents a better approximation of the CTE.  

This solution could be recursively applied to deal with situations where the car is near perpendicular to the track direction, and the track is curved within the area of search.


-------

<i>Note - I added transform_coords.cpp and transform_coords.h to the MPC project, and added them into CMakeLists.txt so that they would be integrated into the Makefile</i>.
