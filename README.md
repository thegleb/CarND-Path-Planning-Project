# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator to run the project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) in the original repository.  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The planner will receive the car's localization and sensor fusion data, and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Approach and implementation

#### 1. Exploration

I first followed the project FAQ example and wrote similar code to generate a set of next points for the car to follow, along a spline that followed the roadway, with anchor points about 30, 60, and 90 meters in front of the car and an evenly spaced set of x and y coordinates with the distance between them corresponding to the car's current speed (see the `generate_trajectory_for_lane` method in `planner.cpp`)

#### 2. State machines

I decided to model the car as a state machine with the following states:

* KL - Keep lane. Common state for a car that remains within its lane.
* LCL - Lane change left. Used to determine when to initiate a lane change to the left.
* LCR - Lane change right. Same as above, but for lane changes to the right.
* LCIP - Lane change in progress. This is used when the car has already initiated a lane change, and this state is kept until the lane change is completed.

I initially created 2 intermediate states between KL and LCL/LCR called PLCL/PLCR (prepare lane change left/right), but I couldn't get the car to behave properly so I simplified the state machine a bit.

#### 3. Basic algorithm

The pattern is pretty simple:

1) Given the car's current state (initialized to KL), determine the next possible states
2) Create a plan for each state by determining a trajectory as well as a target velocity for each state. The target velocity is basically just "assuming the car is in the target lane, find the nearest other car in front within a certain range and match its speed, else pick the maximum legal velocity"
3) Using the `sensor_fusion` observations around the car, make predictions about the other cars assuming they're a) moving at constant velocity and b) staying in their own lane
4) Using a series of cost functions and cost function weights, calculate a cost value for each of the possible plans from (1)
5) Select the lowest-cost plan by choosing the lowest value from (4)
6) Update the car's target lane and target velocity using the chosen state from (5) 

The core logic for this is in the `choose_next_plan` method inside `planner.cpp`.

The other important step is also number 7: adjust ego car velocity to match target velocity. For this I use a pretty simple approach - convert the maximum allowed acceleration to mph and either add or subtract `max_a * delta_t`, as illustrated in the `get_updated_speed` method in `planner.cpp`:
```
return current_v + mph_from_mps(MAX_A * (delta > 0 ? 1 : -1)) * D_T;
```

I initially tried to combine trajectory generation with velocity updates, but wasn't able to get it to work, thus I split it into a separate update step combined with the target velocity determined for each possible `Plan`.

#### 4. Cost functions

My cost functions strive to maximize velocity and are not very optimized for nuanced maneuvers:

* `maximize_speed` - The target velocity should be as close to the maximum velocity as possible. If the delta from maximum velocity is larger, the cost is higher.
* `choose_fast_lane` - If the car's current velocity is lower than the lane velocity (as determined by the velocity of the nearest car in front), then return 0; otherwise the cost increases the slower the lane is compared to the ego vehicle.
* `valid_lane` - Return a very high cost if the new lane value is outside allowed boundaries. This makes sure the car never tries to change lanes to the right when it is in the right lane, or change to the left from the left lane.
* `dont_cut_off_faster_cars` - If there is a faster-moving car behind, then increase cost based on the delta of the velocities. 
* `dont_merge_when_car_nearby` - Increase cost if there is another car within a certain range in front or behind the ego vehicle.

#### 5. Cost function weights

For the most part I wrote the cost functions in a way that each one would return a value from ~0 to ~1 so that it would be easy to assign weights. The one exception is the `dont_cut_off_faster_cars` function where I gave up and left it as a pure velocity delta (which has a max theoretical value of `TARGET_V` of 49.5), at least as of time of writing this.

I chose these weights (listed in order of appearance on the cost function list above): 
```
vector<float> weight_list = {1000, 3000, 1000000, 10000, 100000};
``` 

The most important cost functions that had to override everything were `valid_lane` and `dont_merge_when_car_nearby`. `valid_lane` is self-explanatory, and thehigh value for  `dont_merge_when_car_nearby` came out of frustration when the first bunch of iterations kept knocking other cars off the road (see [one such illustrative example](https://www.youtube.com/watch?v=A3Q9WGUf7rA)).

#### 6. Shortcomings

There are a few shortcomings of this implementation I know of:
* The planner assumes all the cars around are moving at constant velocity and are not switching lanes. To solve this I'd probably do a combination of tracking velocities over time to determine acceleration as well as running something like a naive Bayes classifier on their velocities to see whether they are potentially changing lanes or not.
* The trajectory is generated in a fairly naive way, so it doesn't have a good way to guard against extreme jerk/acceleration values that can occur sometimes. I adjusted the numbers in a way that minimizes the occurrence of these, but a better solution would do the legwork of calculating the projected acceleration/jerk values and adjusting the trajectories as needed.
* Similarly the accelerate/brake functions of the ego car are pretty naive and cannot react quickly enough to cars cutting in front, so there is a risk of collision in this case. Luckily this doesn't seem to happen too often in the simulator.
* There is a small bug where the car cannot stop, because if it does, then the code crashes because 2 points with the same `x` coordinate (of 0) are fed to the spline generator, and an assertion fails crashing the planner.

There are probably some others, but these are off the top of my head as I wrote the code. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains `[x, y, s, dx, dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

```
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH
```
#### Previous path data given to the Planner

```
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
```

#### Previous path's end s and d values 

```
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value
```

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

```
["sensor_fusion"] A 2d vector of cars moving in the same direction [id, x, y, vx (m/s), vy (m/s), s, d]. 
```

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
