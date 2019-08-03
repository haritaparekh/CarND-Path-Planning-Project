## Path Planning Project

The goal of this project is to safely navigate around a highway in a simulator based environment with a speed limit as 50MPH. Other traffic is at speed +/-10 of the road speed. The car should try to go as close as possible to the 50 MPH speed limit, change lanes and pass slower traffic when possible without hitting any other car on the road. 

The simulator provides car's localization data and sensor fusion data - list of all other car's attributes on the same side of the road. All this data is processed and appropriate action is taken in 3 steps - Prediction, Behavior and Planning. Following sections explain each step in detail. I used code from lessons and project Q&A to start with.

### 1. Prediction

In the prediction step, sensor fusion data is processed to find out position of all other cars on the road. Based on that, closest cars in each lane and their distance from our car is estimated. This is implemented from [line 110 to 152](https://github.com/haritaparekh/CarND-Path-Planning-Project/blob/master/src/main.cpp#L110) in the code.

### 2. Behavior

Once closest cars in each lane are found, model finds out the best lane to drive with the help of state machine. State machine has three states - state 0, state 1, state 2. Each state represents a lane. Model can look up to 100 meters ahead of the car and predicts if lane change is required. Following image show lane change:

![alt text](output/lane_change.gif "output_video")

If another car is driving ahead in the same lane and lane change is not possible, model reduces the velocity gradually and adopts speed of the car in front. Following image shows how this is handled:

![alt text](output/speed_change.gif "output_video")

If current lane is any of the side lane i.e. lane 0 or lane 2, and if another side lane is the best choice for target lane, model can switch lane from one side lane to another side lane via middle lane. Following image shows changing multiple lanes:

![alt text](output/multilane_change.gif "output_video")

Before changing the lane, model also checks if there is safe distance from the car coming from behind in the target lane. This state machine is implemented from [line 155 to 242](https://github.com/haritaparekh/CarND-Path-Planning-Project/blob/master/src/main.cpp#L155) in the code.

### 3. Trajectory Planning

Once target lane and speed is decided, trajectory is calculated using spline. This is implemented from [line 258 to 364](https://github.com/haritaparekh/CarND-Path-Planning-Project/blob/master/src/main.cpp#L258) in the code. Car's current state or a couple end points from car's previous path are used as a reference to create new path. This ensure smooth transitioning if changing lanes. Widely spaced waypoints are calculated based on target trajectory. These points are converted to car's local coordinates by shifting car reference angle to 0 degrees and then used as anchor points to spline. Spline points are broken up in such a way that car can travel at desired reference velocity. These points are then converted back to map points and sent to simulator.

### Output

As a result -
- The car could drive at least 4.32 miles without incident.
- The car could drive according to the speed limit.
- The car did not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
- The car never came into contact with any of the other cars on the road.
- The car stayed in its lane, except for the time between changing lanes.
- The car could change lanes whenever required and possible.

![alt text](output/output_video.gif "output_video")

Here is link to full [video result](output/output_video.mp4)

