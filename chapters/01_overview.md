# I. Overview

This section gives the overview of ORB-SLAM2 system [@Mur-Artal2015 
([link](https://ieeexplore.ieee.org/document/7219438))] (currently monocular 
only) and describes the general procedures inside each part of the system.


## 1. System Structure
- Monocular ORB-SLAM:  
  ![sys_struct](images/ch01/system_overview.jpg){ width=75% }
- 3 parallel threads
  (1) Tracking thread
      - Feature selection: ORB [@Rublee2011 ([link](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.370.4395&rep=rep1&type=pdf))]
      - Feature extraction: grid-based for consistent feature extraction within 
        any image grid
      - Map initialization
  (2) Local Mapping thread
  (3) Loop Closing thread


## 2. General Procedures

### 2.1 System Initialization
(1) Parse offline configurations
    - Camera intrinsics
    - Feature extraction parameters
    - Viewer parameters
(2) Load bag-of-words vocabulary [@GalvezTRO12 
    ([link](http://webdiis.unizar.es/~dorian/papers/GalvezTRO12.pdf))] 
    trained offline
(3) Create threads
    - Tracking thread (main thread)
    - Local Mapping thread
    - Loop Closing thread
    - Viewer thread (optional)

### 2.2 Tracking Thread

For each input frame,

(1) Convert input image frame to grayscale
(2) Construct necessary info for current frame
    a) Extract ORB features
    b) Undistort keypoint coordinates given camera intrinsics
    c) Assign ORB features to each image grid
    d) Assign other necessary info
(3) Do map initialization if it is not initialized
    a) If there's no initial frame, set current frame as initial frame 
       and proceed to next frame
    b) Match ORB features between initial frame and current frame
    c) If there're not enough matched keypoints, discard initial & current 
       frame info and proceed to next frame
    d) Set up RANSAC framework, and for each set of keypoints, compute 
       homography \( \V{H} \) and fundamental matrix \( \V{F} \) simultaneously
    e) Select the best \( \V{H} \) and the best \( \V{F} \) based on minimal
       symmetric transfer errors of the matched keypoints between the 2 frames
    f) Compute a score according to the best \( \V{H} \) and \( \V{F} \), 
       and use either \( \V{H} \) or \( \V{F} \) to reconstruct 3D points 
       and camera pose
    g) Create initial map based on 3D points and camera pose
(4) Track current frame and compute camera pose if map is initialized
    a) Do pose estimation if tracking is successful for last frame
    b) Do global relocalization if tracking is failed for last frame
    c) Track local map: find more map point correspondences in a local map 
       if operation in a) or b) is successful, and optimize pose based on
       local map correspondences
    d) Update velocity data if tracking is successful for current frame
       after operations in a)/b) and c)
    e) Check whether set current frame as a new keyframe if tracking is 
       successful
    f) Discard keypoint outliers of current frame if tracking is successful
    g) If tracking is failed soon after map initialization (e.g., inserted
       keyframe is not larger than 5), reset all info and restart map 
       initialization
(5) Return current camera pose **Tcw** (\( [\V{R} | \V{t}]_{3 \times 4} \))

### 2.3 Local Mapping Thread

The local mapping thread will always check whether there're new keyframes
to be processed, and add new keyframes and new map points to the 
covisibility graph and map, respectively, as long as the ORB-SLAM2 system 
is not finished running.

The following is the main procedure for each loop in the thread:

(1) Do not accept new keyframes;
(2) Check whether there're new keyframes to be processed;
(3) If new keyframes do not exist, go to the next step, otherwise, 
    for the oldest new keyframe in the list:
    a) Compute bag-of-words for current new keyframe;
    b) Associate map points to the new keyframe;
    c) Update links in the covisibility graph;
    d) Insert the new keyframe to the map;
    e) Cull map points;
    f) Triangulate new map points;
    g) Search neighbour keyframes of current new keyframe if there're 
       no new keyframes to be processed for more keypoint matches, 
       and update map points and connections in the covisibility graph;
    h) Perform *local* bundle adjustment if there're no new keyframes to
       be processed, and then cull redundant local keyframes;
    i) Add current new keyframe to loop closing thread.
(4) Stop processing if user requests, and exit the loop if the system is
    finished;
(5) Reset the thread if user requests;
(6) Exit the loop if the system is finished;
(7) Continue accepting new keyframes.

### 2.4 Loop Closing Thread

The loop closing thread will always check whether there're new keyframes
to be processed, and detect whether the map contains loop. If a loop is
detected, it will perform loop fusion and pose graph optimization
to eliminate the loop, as long as the ORB-SLAM2 system is not finished 
running.

The following is the main procedure for each loop in the thread:

(1) Check whether there're new keyframes to be processed;
(2) If new keyframes do not exist, proceed to next step. Otherwise,
    for the oldest new keyframe in the keyframe queue:
    a) Detect whether there's a loop in the map based on the current 
       new keyframe;
    b) If a loop is detected, compute similarity transformations
       (\( Sim(3) \)) from current keyframe to each loop candidate keyframe, 
       and check whether there are enough keypoint matches to accept a loop;
    c) If a loop is confirmed, it will be corrected.
(3) Reset the thread if user requests;
(4) Exit the loop if the system is finished.


\newpage
