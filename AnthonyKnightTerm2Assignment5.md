## Model Predictive Controller (MPC) Project
### Writeup by Tony Knight - 2017/10/03


---

#### <i>Note - I added transform_coords.cpp and transform_coords.h to the project, and added them into CMakeLists.txt so that they would be integrated into the Makefile</i>.

### Kinematic Car Model
The model predictive controller utilizes a simplified Kinematic model, as opposed to a full dynamic model of the car.  It does not take into account factors like air resistance, road / tire friction, lateral forces, and gravity.  The model state consists of the car location (x,y), speed, direction, steering angle  

the car can be controlled by two actuators, steering and throttle.  The throttle rate does not correspond exactly to an acceleration rate, as a given throttle rate in the simulator will not be able to accelerate a car beyond a speed (in mph) of 100 times the throttle rate.

### CTE

The generalized solution to calculate the cross track error is to take the y intercept of the polyfit waypoints curve using the "car" co-ordinates (where the car is at 0,0 and is heading along the x-axis).  This approximation becomes invalid as the difference in the directions of the polyfit line and of the car diverge.  This could occur if the car spins out along the waypoint line.

I attempted to calculate a slightly better approximation of the CTE by 
1) locating two points on the polyline close to the CTE, 
2) calculating the angle of the line at that location (in Car co-ordinates),
3) transforming the location and direction of the co-ordinates so that the polyfit line lay along the x axis near the CTE.  
The transformed y co-ordinate of the car should represent a better calculated value of CTE.


### Number and Duration of Timesteps in Model
I found different solutions using 16 timesteps of 0.05 s and 8 timesteps of 0.1 both resulting in similar performance which could make it safely around the track with top speeds of about 60 mph.

predict 8 timesteps of 0.1s

cte_factor = 750
epsi_factor = 1200
v_factor = 10.0
steer_factor = 0.0 
accel_factor = 25.0
delta_factor = 0.0
delta_a_factor = 10.0
delta_cte_factor = 7500.0

As the horizon was increased, the car did 


```c
      fg[0] += cte_factor * squared(vars[cte_start + t] - ref_cte)*(t+5);//minimize cross track error
      fg[0] += epsi_factor * squared(vars[epsi_start + t] - ref_epsi)*(t + 2);//minimize direction error
      fg[0] += v_factor * squared(vars[v_start + t] - ref_v);//try to maintain a target speed
    }
```


```c
    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += steer_factor * squared(vars[delta_start + t] - ref_delta)*(t + 3);// (t + 5);  //minimize steering angles
      fg[0] += accel_factor * squared(vars[a_start + t] - ref_a);// (t + 5);  //minimize use of accelerator/brake
    }
```

```c
    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++)
    {
      fg[0] += delta_factor * squared((vars[delta_start + t + 1] - vars[delta_start + t]) - ref_delta_delta);// (t + 5); //remove jerkiness in steering
      fg[0] += delta_a_factor * squared((vars[a_start + t + 1] - vars[a_start + t]) - ref_delta_a);// (t + 5); //remove jerkiness in acceleration/braking
    }
```

```c
    for (t = 1; t < N - 1; t++) //do't worry if start of 
    {
      fg[0] += delta_cte_factor * squared((vars[cte_start + t + 1] - vars[cte_start + t]) - ref_cte)*(t+3);// (t + 5); //minimize change in cte (similar to steer reduction?
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


