# **Structure from Motion** (1b) - Probabilistic Robotics Project

## **Main steps**

### 0) Translations initialization
Least Squares to estimate the *position* of each camera.
Epipolar constraint as *error function* using direction vectors of mathching keypoints.

### 1) Triangulation
To get the 3D position of each landmark.
Use direction vectors to calculate line intersection points.

### 2) Bundle Adjustment 
Final refinement

## **Final output**
- Global position of each camera
- Global position of each landmark
- Error values