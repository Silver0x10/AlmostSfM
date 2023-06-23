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



# **Hot to Run**
```bash
./build/executables/sfm dataset_and_info/dataset.txt out/cameras.txt out/landmarks.txt dataset_and_info/GT_landmarks.txt
```

## **Final output**
- Global position of each camera
- Global position of each landmark
- Error values