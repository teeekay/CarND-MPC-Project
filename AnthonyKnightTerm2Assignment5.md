## Model Predictive Controller (MPC) Project
### Writeup by Tony Knight - 2017/10/03

---


<img src="https://raw.githubusercontent.com/teeekay/CarND-MPC-Project/master/examples/MPC_Screenshot.png?raw=true"  width=600>

<i><u>Figure 1: Snapshot from MPC Video</u></i>

---

### Running MPC


The model can be run from the build directory by calling 
```sh
$./mpc
```
which will execute the model using a target velocity of 70 mph and 14 timesteps of 0.05s.

Alternatively, you can specify a different target velocity like this:

```sh
$./mpc 85.
```
You can also specify the timestep duration and number of timesteps from the CLI as follows:
```sh
$./mpc 55 0.1 12
```

With the default settings, the car can safely go around the track at speeds of up to 75 mph.  At 75 mph and beyond the car will often run up against the concrete ledge.  The car will generally crash within a couple of laps at speeds of 85 mph and beyond. 


### Kinematic Car Model
The model predictive controller utilizes a simplified Kinematic model, as opposed to a full dynamic model of the car.  It does not take into account factors like air resistance, road / tire friction, lateral forces, and gravity.  The model state consists of the car location (x,y), speed, direction, steering angle  

The car can be controlled by two actuators: throttle and steering.  The throttle rate does not correspond exactly to an acceleration rate, as a given throttle rate in the simulator will not be able to accelerate a car beyond a speed (in mph) approximately 100 times the throttle rate.  The steering is limited to +/- 25 degrees which is input as a control of -1 to 1.


### Number and Duration of Timesteps in Model
I found different solutions using time intervals of 0.05 s and 0.1 s, both resulting in similar performance which could make it safely around the track with top speeds of about 50 mph without running onto the concrete verges.  I found a range of timesteps where the car could make it around the track at speeds of 

### <b>Table 1:</b>
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
 - works ok when using 10 to 20 timesteps at desired speeds below 50 mph


### <b>Table 2:</b>
|Timestep | 0.05s|
|:---|:--|
|cte_factor | 7,500|
|epsi_factor | 500|
|v_factor | 5|
|steer_factor | 0 |
|accel_factor | 10|
|delta_factor | 1|
|delta_a_factor | 3|
|delta_cte_factor | 750,000|
 - work oks for 18 to 28 timesteps at desired speeds below 50 mph

generally as the number of timesteps is reduced below the acceptable range, the car drives in a straighter line, and is less affected by 


As the horizon was increased, the car did 


```c
      fg[0] += cte_factor * squared(vars[cte_start + t] - ref_cte);    //minimize cross track error
      fg[0] += epsi_factor * squared(vars[epsi_start + t] - ref_epsi); //minimize direction error
      fg[0] += v_factor * squared(vars[v_start + t] - ref_v);          //try to maintain a target speed
    }
```


```c
    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += steer_factor * squared(vars[delta_start + t] - ref_delta); //minimize steering angles
      fg[0] += accel_factor * squared(vars[a_start + t] - ref_a);         //minimize use of accelerator/brake
    }
```

```c
    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++)
    {
      fg[0] += delta_factor * squared((vars[delta_start + t + 1] - vars[delta_start + t]) - ref_delta_delta); //remove jerkiness in steering
      fg[0] += delta_a_factor * squared((vars[a_start + t + 1] - vars[a_start + t]) - ref_delta_a); //remove jerkiness in acceleration/braking
    }
```

```c
    for (t = 1; t < N - 1; t++) //do't worry if start of 
    {
      fg[0] += delta_cte_factor * squared((vars[cte_start + t + 1] - vars[cte_start + t]) - ref_cte); //minimize change in cte between timesteps - straighten curve
    }
```

I used a timestep of 0.05 with a 

I found that although the simulator was useful to observe patterns, it is not particularly good way to optimize 

I found that on some timesteps the polynomial fitted to the waypoints "jogged" to one side at the end of the line adjacent to the car.  this resulted in the car moving to the side at this point also. I attempted to resolve this "feature" by adding another control waypoint at the start of the line (storing the first waypoint between program iterations, and then inserting the initial waypoint).  However, fitting the curve to an additional waypoint caused problems when the curve became more complex.  I "fixed" this by removing the final set of waypoints so the curve only had to fit 6 waypoints again.  The final implementation used a 3rd order polynomial to fit to the waypoints, however, the code in was written so flexibly so that a higher order implementation could be used.  the 

The solution I implemented to get around a known 100 mS delay in applying actuator values was to calculate the approximate pose of the car 100 mSecs into the future based on the known location, speed, direction and steering angle of the car (disregarding effects of acceleration or changes in steering) and then to use that position to calculate the relative locations of the waypoints and the cte before solving.  A solution could also have been implemented within the model solver.  However, this would have relied on the latency being a multiple of the model step length to get a good value being multiples

``` c
          px = px + (v * CppAD::cos(psi) * (latency_delay/1000.0));
          py = py + (v * CppAD::sin(psi) * (latency_delay/1000.0));
          psi = psi - (v / Lf * steer / (0.436332 * Lf) * (latency_delay/1000.0));
```

## Additional Experiments
I attempted to see if increasing the cost of actions in the future would result in a quicker response

### CTE

The generalized solution to calculate the cross track error is to take the y intercept of the polyfit waypoints curve using the "car" co-ordinates (where the car is at 0,0 and is heading along the x-axis).  This approximation becomes invalid as the difference in the directions of the polyfit line and of the car diverge.  This could occur if the car spins out along the waypoint line.

I attempted to calculate a slightly better approximation of the CTE by 
1) locating two points on the polyline close to the CTE, 
2) calculating the angle of the line at that location (in Car co-ordinates),
3) transforming the location and direction of the co-ordinates so that the polyfit line lay along the x axis near the CTE.  
The transformed y co-ordinate of the car should represent a better calculated value of CTE.

-------

<i>Note - I added transform_coords.cpp and transform_coords.h to the MPC project, and added them into CMakeLists.txt so that they would be integrated into the Makefile</i>.

