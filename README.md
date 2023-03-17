# **Structure from Motion** (1b) - Probabilistic Robotics Project

## **Main steps**
### 0) Essential Matrices
8-point algorithm to extract the essential matrix E for each pair of cameras.
SVD decomposition -> R_ij|t_ij

### 1) Global translation problem
Least Squares to estimate the *position* of each camera.

Epipolar constraint as *error function*.

Translation vectors from essential matrices as *measurements*.

### 2) Triangulation
To get the 3D position of each landmark

### 3) Bundle Adjustment 
Final refinement


## **Final output**
- Global position of each camera
- Global position of each landmark