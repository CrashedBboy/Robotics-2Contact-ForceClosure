# Robotics-2Contact-ForceClosure
In this project, we create an algorithm in MATLAB that determines if a 2-Contact Grasp (using hard contacts) satisfies force closure.  


Visualization of friction cone
![Friction Cone](https://github.com/CrashedBboy/Robotics-2Contact-ForceClosure/blob/main/OutputPlots/FrictionCone1.png?raw=true)

## Algorithm Steps
1. Build friction cone boundary vectors
2. Construct grasp wrench space (GWS)
3. Determine force-closure: Check if CoM is inside of the convex hull from friction cones
