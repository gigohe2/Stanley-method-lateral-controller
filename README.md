# Stanley-method-lateral-controller

## 1. Introduction
It is implementation of lateral controller for autonomous driving car using stanley method. 
Stanley method is better than pure pursuit in that it can reduce cross-tracking-error and follow the trajectory simultaneously.

## 2. Implementation
In real environment, i designed stanley method lateral controller for mini autonomous driving car.

To make a trajectory, i used the RTK sensor which can provide accurate GPS position.
And then, I applied UTM coordinates to get meter scale position of mini autonomous driving car.

When we apply stanley method, we have to know the status of the car and the slope of the trajectory.
Here is the list of used status and variables.
1. Position of car
2. Orientation(heading) of car
3. Target point to follow
4. Slope of the trajectory at target point which obtained at 3

By using above information, we can apply stanley method.
![image](https://github.com/gigohe2/Stanley-method-lateral-controller/assets/59073888/c6af9b87-ad29-44ae-a704-3b34e4416c94)

![image](https://github.com/gigohe2/Stanley-method-lateral-controller/assets/59073888/3e45804b-1b44-44b8-a593-26915722f424)

Psi is the heading error between heading of car and slope of trajectory.
Vector x is the vector from front wheel center to the closest target point at trajectory.
k is the stanley constant which determines the tracking performance. When k is high, steering angle is more aggresive to close the gap between car and trajectory.

It is not necessary to apply PID controller following stanley controller because stanley controller provides the steering angle which is accurate steering angle to track trajectory.\

