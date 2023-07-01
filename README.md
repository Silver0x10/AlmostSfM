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
From the *build* directory:
```bash
 ./executables/sfm ../dataset_and_info/dataset.txt ../dataset_and_info/GT_landmarks.txt ../out  0
```
The arguments are (in order):
1) Dataset path
2) Landmarks ground truth path
3) Output directory
4) Bundle Adjustments rounds

## **Final output**
- Global position of each camera (**out/cameras.txt**)
- Global position of each landmark (**out/landmarks.txt**)
- Camera positions error values (**out/camera_position_errors.txt**)
- Camera rotations error values (**out/camera_rotation_errors.txt**)
- Landmarks RMSE with the used Sim(3) (**out/landmarks_error.txt**)