## Model Predictive Controller (MPC) Project
### Writeup by Tony Knight - 2017/10/03

---


<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/MPC_Screenshot2.png?raw=true"  width=700>

<i><u>Figure 1: Snapshot from MPC Video</u></i>

---

### Running MPC


The model can be run from the build directory by calling 
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

The car can be controlled by two actuators: throttle and steering.  The throttle rate does not correspond exactly to an acceleration rate, as a given throttle rate in the simulator will not be able to accelerate a car beyond a speed (in mph) approximately 100 times the throttle rate.  negative throttle rates act as reverse acceleration which is equivalent to braking.  The steering is limited to +/- 25 degrees which is input as a control of -1 to 1.

In our model, the following equations are used to determine the new position, speed, and direction of the car at time t which is one interval dt after the last known position. Lf is a constant which is the distance from the front of the car to the centroid of the car mass.

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/stateupdate.png?raw=true"  width=250>



### Number and Duration of Timesteps in Model

I initially tested the use of a time interval of 0.05 s and tuned other parameters to allow the car to go around the track at speeds of 40 mph.  I found that the optimal value of N (number of timesteps) to use was 22 at this speed resulting in a horizon of about 1.1 seconds.  I tested these parameters when switching to using a timestep of duration of 0.1s, and found I had to retune the parameters and reduce the number of timesteps to 12.  I generally found that reducing the number of timesteps below the optimal rate made the car go straighter but potentially in the wrong direction, whereas increasing the number of timesteps increased wobbles particularly after exiting a curve.

After tuning other parameters extensively I developed the following relationship between the optimal number of timesteps and the desired velocity when using a timestep of 0.05 seconds duration (shown in figure below).

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/OptimalTimesteps.png?raw=true"  width=400>

Interestingly this produced horizon lengths which increased linearly to about 25 m for speeds below 50 mph, and then remained constant near this length until the speed increased beyond 85 mph (as shown in the diagram below)

<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/OptimalHorizonDistance.png?raw=true"  width=400>



### <b>Table 1:</b>
|Timestep | 0.05s|
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

|Timestep | 0.1s|
|:---|:-|
|cte_factor | 3,750|
|epsi_factor | 2,400|
|v_factor | 10|
|steer_factor | 0 |
|accel_factor | 125|
|delta_factor | 0|
|delta_a_factor | 10|
|delta_cte_factor | 22,500|



### Solver Equations

```C++
    ref_v = desired_velocity * 1609 / 3600;
    ref_a = desired_velocity / 100;
```

```C++
// Minimize tcross track error, incorrect direction, and velocity.
    for (t = 0; t < N - 1; t++) {
      fg[0] += cte_factor * squared(vars[cte_start + t] - ref_cte);    //minimize cross track error
      fg[0] += epsi_factor * squared(vars[epsi_start + t] - ref_epsi); //minimize direction error
      fg[0] += v_factor * squared(vars[v_start + t] - ref_v);          //try to maintain a target speed
    }
```


```C++
    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += steer_factor * squared(vars[delta_start + t] - ref_delta); //minimize steering angles
      fg[0] += accel_factor * squared(vars[a_start + t] - ref_a);         //minimize use of accelerator/brake
    }
```

```C++
    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++)
    {
      fg[0] += delta_factor * squared((vars[delta_start + t + 1] - vars[delta_start + t]) - ref_delta_delta); //remove jerkiness in steering
      fg[0] += delta_a_factor * squared((vars[a_start + t + 1] - vars[a_start + t]) - ref_delta_a); //remove jerkiness in acceleration/braking
    }
```

```C++
    for (t = 1; t < N - 1; t++) //do't worry if start of 
    {
      fg[0] += delta_cte_factor * squared((vars[cte_start + t + 1] - vars[cte_start + t]) - ref_cte); //minimize change in cte between timesteps - straighten curve
    }
```


### Waypoint Polynomial fit

I initially used a third order polynomial fit to the 6 waypoints provided using the provided polyfit function.  However, I found that on some timesteps the polynomial line "jogged" to one side at the end of the line close to the car.  This resulted in the car moving to the side at this point also. I attempted to resolve this "feature" by inserting an earlier control waypoint at the start of the line (storing the first waypoint between program iterations, and then inserting as the initial waypoint).  However, fitting the curve to an additional waypoint caused problems when the curve became more complex.  I "fixed" this by removing the final set of waypoints so the curve only had to fit 6 waypoints again.  

During tuning I ran a test where I used 2nd order polynomial fit instead.  This provided better results on the track, possibly because each curve segment on the track can easily be modelled by a 2nd order polynomial, and the solver does not try to compensate for future curves in the opposite direction to the upcoming curve.


### Latency

The solution I implemented to get around a known 100 mS delay in applying actuator values was to calculate the approximate pose of the car 100 mSecs into the future, based on the known location, speed, direction and steering angle of the car (and disregarding effects of acceleration or changes in steering during this interval).  This projected position is then used to calculate the relative locations of the waypoints and the cte before solving.  

``` C++
          px = px + (v * CppAD::cos(psi) * (latency_delay/1000.0));
          py = py + (v * CppAD::sin(psi) * (latency_delay/1000.0));
          psi = psi - (v / Lf * steer / (0.436332 * Lf) * (latency_delay/1000.0));
```

A solution could also have been implemented within the model solver where actuator settings are not applied until after 0.1 seconds.  However, this would have relied on the latency being a multiple of the model step length to get good results.


## Additional Experiments
I ran a couple of tests to see if increasing the cost of actions further in the future would result in quicker responses.  I multiplied the cost of state and actuator errors by (t + tunable_constant), where the tunable constant was an integer between 2 and N/2.  The initial tests did not produce observably better results, so I abandoned this approach.

### Curve Direction and Sharpness

I decided that the optimal path of the car (In order to allow it to make it around the track at higher speeds) would be to the inside of the curve produced by the waypoints.  I realized that the sign and magnitude of the second order coefficient of the polyfit line described the direction and sharpness of the curve.  I used this value to calculate reference CTE values for the solver cost calculations, and tuned these equations based on the speed of the car as follows.

```C++
   //ref_cte - adjust ref_cte so that predicted path should be biased to inside of waypoint curve
    // optimal value needed to increase with velocity (also inversely to number of timesteps) 
    if (desired_velocity > 70)
    {//Above 70 mph
      ref_cte = (-coeffs[2]+0.0013) * (620 + 30 * (desired_velocity - 70));//25
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


### No braking


### CTE

The generalized solution to calculate the cross track error is to take the y intercept of the polyfit waypoints curve using the "car" co-ordinates (where the car is at 0,0 and is heading along the x-axis).  This approximation becomes invalid as the difference in the directions of the polyfit line and of the car diverge.  This could occur if the car spins out along the waypoint line.

I attempted to calculate a slightly better approximation of the CTE by 
1) locating two points on the polyline close to the CTE, 
2) calculating the angle of the line at that location (in Car co-ordinates),
3) transforming the location and direction of the co-ordinates so that the polyfit line lay along the x axis near the CTE.  
The transformed y co-ordinate of the car should represent a better calculated value of CTE.  

This solution could be recursively applied to deal with situations where the car is near perpendicular to the track direction, and the track is curved within the area of search.


-------

<i>Note - I added transform_coords.cpp and transform_coords.h to the MPC project, and added them into CMakeLists.txt so that they would be integrated into the Makefile</i>.

