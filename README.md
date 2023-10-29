# **Structure from Motion** (1b) - Probabilistic Robotics Project

## Step 0 - Translations initialization
Least Squares to estimate the *position of each camera* (MultiPoseRegistration-like) using the *epipolar constraint as error function* and the estimated global camera orientations from project 1a.<br/>

The initialized translations are extracted from the null space of the H matrix of the LS problem selecting one of the rightmost columns of the V matrix of its SVD decomposition.These rightmost column in fact form a basis of the H matrix null space. [see **src/init_translation.\***] <br/>

The relative position between each pair of cameras is required for epipolar constraint evaluation. It is estimated through the following procedure [see **src/relative_position_calculator.\***]:
1) *8-point-algorithm* to estimate the *essential matrix* using corresponding direction vectors of the two cameras
2) Relative *position extraction* from the essential matrix. It is selected among the possible solutions tanking the one with the highest number of points in front of the camera. This has been done triangulating the two corresponding direction vectors for each landmark and then projecting it along the z-axis of the camera (3rd row of the analyzed rotation matrix). 


## Step 1 - Landmarks Triangulation
To get the 3D *position of each landmark* using *direction vectors* to calculate line intersection points [see the [reference](https://silo.tips/download/least-squares-intersection-of-lines) (Section 3) and **src/triangulation.\***]. <br/>

Each position is calculated solving a LS problem using all the direction vectors corresponding to the same landmark coming from all the available cameras.

## Step 2 - Bundle Adjustment
Final refinement of camera poses and landmarks position formulated as a LS problem [see **src/bundle_adjustment.\***] trying to minime the following errors:
- **Pose-Pose error**: epipolar constraint between each camera pair (as in step 0)
- **Pose-Landmark error**: cross product between measured and predicted direction vectors


## Step 3 - Landmarks Registration
Estimation of the Sim(3) tranformation between estimated and reference landmark positions [see the [reference](https://gitlab.com/grisetti/probabilistic_robotics_2022_23/-/blob/main/slides/probabilistic_robotics_23b_registration_on_a_manifold.pdf) and **src/icp_3d.\***]

**NOTE_1**: This transformation for the moment has been used only in the landmark evaluation step.

## Step 4 - Evaluation methods
Camera positions, camera orientations and landmark positions are evaluated independently comparing them with the Ground Truth values through the following procedures [see **src/evaluation.\***]:

### Camera positions:
For each camera pair:
1) Compute the norm of the estimated relative position of camera j with respect to camera i
2) Compute the norm of the reference relative position of camera j with respect to camera i
3) Compute the ratio between these two values

The value of this ration should be the same for all the pose pairs.

### Camera orientations:
For each camera pair:
1) Compute the delta rotation between the two estimated orientations
2) Compute the delta rotation between the two reference orientations
3) Compare the two delta rotations: *trace(eye(3) - R_delta^T * R_delta_gt)*

### Landmark positions:
1) Transform each landmark position using the Sim(3) transformation estimated in Step 3
2) Compute the Root Mean Squared Error between estimated and reference landmark positions


## Step 5 - Visualization
<div align="center"> <img src="out/visualization.png" width="70%"/> </div>
At the end a window like that in the above image should appear, showing
- Estimated camera positions in blue
- Estimated landmark positions in red
- GT camera positions in violet
- GT landmark positions in green

<br/> 

**NOTE_2**: wrong estimated positions in the above image :(

**NOTE_3**: in the current version of *visualize()* in *executables/structure_from_motion* the code regarding the visualization of GT values has been disable to focus on the estamated ones

# **Hot to Run**
From the *build* directory, execute:
```bash
 ./executables/sfm ../dataset_and_info/dataset.txt ../dataset_and_info/GT_landmarks.txt ../out  5
```
The arguments are (in order):
1) Dataset path
2) Landmarks ground truth path
3) Output directory
4) Bundle Adjustments rounds

The output of the above command can be found in **out/terminal.txt**

## **Final output**
- Global position of each camera (**out/cameras.txt**)
- Global position of each landmark (**out/landmarks.txt**)
- Camera positions error values (**out/camera_position_errors.txt**)
- Camera rotations error values (**out/camera_rotation_errors.txt**)
- Landmarks RMSE with the used Sim(3) (**out/landmarks_error.txt**)

# Tests

- [X] icp_3d (```build/executables/test_icp_3d <Dataset Path> <GT Landmarks Path>```)
  - [X] v2RPY
  - [X] Sim3
    - [X] * operator
    - [X] boxplus
<div align="center"> 
  <img src="out/icp_test_visualization.png" width="70%"/> 
  <img src="out/icp_test_terminal.png" width="70%"/>
</div>
  
- [X] triangulation (```build/executables/test_triangulation <Dataset Path> <GT Landmarks Path>```)
<div align="center"> <img src="out/triangulation_test_visualization.png" width="70%"/> </div>

- [X] calculate_relative_position (```build/executables/test_relative_pos <Dataset Path> <GT Landmarks Path>```)
  - [X] eight_point_algorithm
  - [X] extract_t
<div align="center"> <img src="out/relative_position_test_terminal.png" width="70%"/> </div>

- [ ] init_translations

- [ ] bundle_adjustment
