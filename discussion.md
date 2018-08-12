## Project Write-up
---

### Model Predictive Control (MPC)
MPC is an optimization model that predicts the actuator values to be used at each moment. In ths optimization model, we have the states in `N` timesteps, then we will find a set of actuator values (for each timestep) that minimize a cost function. The set of actuator values for the very first timestep will be used to control the car.

A state of the car consists of the following values:
* `x`: the position of the car in x-axis (either in map coordinates or in vehicle coordinates).
* `y`: the position of the car in y-axis.
* `psi`: the orientation of the car (in radians).
* `v`: the velocity of the car (in mph).
* `cte`: the cross-track error, measured by the offset of the car position to the road waypoint in y-axis.
* `epsi`: the orientation error, measured by the offset of the car orientation to the desired orientation.

The actuator of the car consists of:
* `delta`: the steering value.
* `a`: the throttle value.

The update equations from one time step to another are:
```
x1 = x0 + v0 * cos(psi0) * dt;
y1 = y0 + v0 * CppAD::sin(psi0) * dt; 
psi1 = psi0 + (v0/Lf) * delta0 * dt;
v1 = v0 + a0 * dt;
cte1 = (f0 - y0) + (v0 * sin(epsi0) * dt);
epsi1 = (psi0 - psides0) + v0 * delta0 / Lf * dt;
``` 
where:
* `Lf`: a constant related to the vehicle physical model.
* `dt`: the duration of a timestep.
* `f0`: the predicted position on y-axis using the fitted polynomial equations from the given waypoints.
* `psides0`: the desired orientation for the given timestep.
* Variables with digit 0 denote values from the previous timestep.
* Variables with digit 1 denote values from the current timestep (that need to be updated).

### N and dt values
I have chosen `N` = 25 and `dt` = 0.05. I haven't tested much with different values for `dt`, but when I set `dt` higher, then the model becomes very erratic. I guess that happens because with longer timestep, we assume the current state values for a longer time, and it leads to more errors, especially with high velocity. 

I feel `N` = 25 is a good value, at least for my desired speed of 40 mph. With this value of `N`, I can be confident with the predicted waypoints, yet the computation can still be done in real-time. 

### Data Preprocessing
The only preprocessing I do is converting the car position and road waypoints from global coordinates to vehicle coordinates. This can be done easily with a transformation matrix. Working in vehicle coordiates simplifies the computation.

### Latency Handling
In my model, latency handling is done via a simulation of the vehicle for the latency period (100 ms). I just assume the velocity and orientation of the car stay the same during the latency period, then I predict the car position when actual steering and throttle values will be applied. That predicted poistion is used in the optimization model (MPC).

