# **Structure from Motion** (1b) - Probabilistic Robotics Project

## **Main steps**

### 0) Translations initialization
Least Squares to estimate the *position of each camera* (MultiPoseRegistration-like).
*Epipolar constraint as error function* using the estimated camera orientataions from project 1b.

### 1) Landmarks Triangulation
To get the 3D *position of each landmark*.
Use *direction vectors* to calculate line intersection points.

### 2) Bundle Adjustment 
Final refinement

## **Final output**
- Global position of each camera
- Global position of each landmark
- Error values