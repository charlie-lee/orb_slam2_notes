# III. Local Mapping Thread

This section gives detail on specific procedures executed on this thread.


## 1. New Keyframe Processing

The new keyframe processing consists of the following procedure:

(1) Compute bag-of-words representation of the new keyframe;
(2) Associate map points and the new keyframe;
(3) Update covisibility graph;
(4) Add new keyframe into the global map.

### 1.1 Association Between Map Points and the New Keyframe

Procedure for each map point in the new keyframe that is observed by 
other keyframes in the map:

(1) Add observation of current new keyframe to the map point;
(2) Update *normal* and depth info of current map point (refer to 
    the same procedure in [initial map creation](#imc) for details);
(3) Compute distinctive descriptor for the map point (refer to 
    the same procedure in [initial map creation](#imc) for details).

### 1.2 Covisibility Graph Update

Covisibility graph is an undirected weighted graph with the following features:

- Vertex/Node: keyframe;
- Edge between 2 keyframes indicates that they share enough observations
  of map points (15 for actual implementation);
- Edge weight: number of common map points.

The update procedure is triggered by the newly processed keyframe, and is
implemented in function `KeyFrame::UpdateConnections()`{.cpp}:

(1) For each map point in the keyframe, find all the other keyframes 
    that observe it, and count the number of common map points between
    the new keyframe and other keyframes;
(2) Add edges between new keyframe and other keyframes that share enough
    map points (at least 15) with edge weight assigned;
(3) If no new edge is added, add an edge between new keyframe and the keyframe
    that shares the most map points with edge weight assigned.
(4) Expand the spanning tree for the keyframes in the map:
    a) Sort the edge weights of the connected keyframes in descending order;
    b) Set the keyframe with the most connections as the parent node 
       in the spanning tree, and add current keyframe as its child.
       - Each keyframe can only be connected to one parent, thus the spanning
         tree expansion procedure is executed only once for each keyframe.


## 2. Redundant Map Point Removal

For each newly added keyframe, a list of recently added new map points
are checked, and redundant map points in the list will be removed by
the following criteria:

(1) The ratio of the number of a map point being an inlier of each keyframe 
    observed it, to the number of the map point being visible from each
    keyframe, is less than a threshold (0.25 in actual implementation);
    - The numerator of the ratio is increased in function
      `Tracking::TrackLocalMap()`{.cpp}, and the increase is triggered
      by the following code:
      
      ~~~{.cpp}
      for (int i=0; i<mCurrentFrame.N; i++) // traverse all keypoint in the frame
          if(mCurrentFrame.mvpMapPoints[i]) // check if it is a map point
              if(!mCurrentFrame.mvbOutlier[i]) // check if it is an inlier
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
      ~~~
    
    - The denominator of the ratio is increased in function
      `Tracking::SearchLocalPoints()`{.cpp} within the function 
      `Tracking::TrackLocalMap()`{.cpp}, and the increase is triggered
      by the following code:
      
      ~~~{.cpp}
      // search for map points in current frame
      for (vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), 
           vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++) {
          MapPoint* pMP = *vit;
          if(pMP && !pMp->isBad())
              pMP->IncreaseVisible();
      }
      // project local map points in frame and check its visibility
      for (vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), 
           vend=mvpLocalMapPoints.end(); vit!=vend; vit++) {
          MapPoint* pMP = *vit;
          // skip map points in current frame
          if(pMP->mnLastFrameSeen == mCurrentFrame.mnId) continue;
          // skip bad point
          if(pMP->isBad()) continue;
          // Project (this fills MapPoint variables for matching)
          if(mCurrentFrame.isInFrustum(pMP,0.5))
              pMP->IncreaseVisible(); // increase the visible count
      }
      ~~~

(2) The current keyframe is at least 2 frames ahead of the keyframe
    where a map point is first observed, and the map point can only be
    observed by no more than 2 frames (in monocular configuration).

The removal procedure is implemented in function 
`LocalMapping::MapPointCulling()`{.cpp} as follows: 
for each map point in the list of recently added map points,

(1) Remove the map point from the list if it is a bad point;
(2) If the criteria above is met, set the map point as a bad point
    and remove it from the map, and then remove the map point from
    the list as well;
(3) If the map point does not meet the criteria above, and
    the current frame is at least 3 frames ahead of the keyframe
    that first observes the map point, remove the map point
    from the list.


## 3. New Map Point Creation

Implemented in function `LocalMapping::CreateNewMapPoints()`{.cpp},
the procedure for each newly added keyframe is as follows:

(1) Retrieve \( k \) (20 in actual implementation) best neighbor keyframes 
    from the covisibility graph;
    - The *best* \( k \) keyframes: sort the keyframes that are connected 
      with the current keyframe by edge weight in descending order, and
      select the first \( k \) of them.
(2) For each neighbor keyframe, compute the fundamental matrix
    between current keyframe and the neighbor keyframe;
(3) Search keypoint matches that fulfill epipolar restriction;
(4) Triangulate qualified keypoint matches to 3D map point;
(5) If the triangulated map point meet various restrictions,
    add it to the map.
    - Update observation info of the map point on current keyframe and
      each neighbor keyframe that successfully triangulated it
      if necessary;
    - For each neighbor keyframe that successfully triangulated the map point,
      add the map point info to it;
    - Compute distinctive descriptor of the map point (check [here](#imc)
      for details);
    - Update *normal* and depth of the map point (check [here](#imc) 
      for details);
    - Add the map point to the list of recently added map points.
    
After selecting \( k \) best neighbor keyframes and before computing fundamental
matrix between the current keyframe and each neighbor keyframe, the baseline
between the 2 keyframes is checked. If it is too small, skip processing current
neighbor keyframe.

### 3.1 Fundamental Matrix Computation

Given the current keyframe \( KF_1 \) with 3D world to 2D image transformation
matrix \( \V{P}_1 = \V{K}_1 \V{T}_1 = \V{K}_1 [ \V{R}_1 | \V{t}_1 ] \), 
and the neighbor keyframe \( KF_2 \) with the corresponding matrix 
\( \V{P}_2 = \V{K}_2 \V{T}_2 = \V{K}_2 [ \V{R}_2 | \V{t}_2 ] \),
the fundamental matrix from \( KF_2 \) to \( KF_1 \), noted as \( \V{F}_{12} \),
is computed as follows (see section 9.2 of [@Hartley2004] for more details):

\[
   \V{F}_{12} = \V{K}_1^{-T} [\V{t}_{12}]_\times \V{R}_{12} \V{K}_2^{-1}
\]

where 
\( [\V{t}_{12}]_\times
   = \begin{bmatrix} t_1 \\ t_2 \\ t_3 \end{bmatrix}_\times 
   = \begin{bmatrix} 
       0 & -t_3 & t_2 \\ t_3 & 0 & -t_1 \\ -t_2 & t_1 & 0 
     \end{bmatrix}
\) is a skew-symmetric matrix, and \( \V{R}_{12} \) and \( \V{t}_{12} \) 
is computed from \( \V{T}_{12} \), the 3D camera to 3D camera coordinate
mapping matrix (from \( KF_2 \) to \( KF_1 \)):
\[
   \V{T}_{12} 
   = \V{T}_1 \V{T}_2^T
   = \begin{bmatrix} \V{R}_1 & \V{t}_1 \\ \V{0}^T & 1 \end{bmatrix}
     \begin{bmatrix} \V{R}_2^T & -\V{R}_2^T \V{t}_2 \\ \V{0}^T & 1 \end{bmatrix}
   = \begin{bmatrix}
       \V{R}_1 \V{R}_2^T & -\V{R}_1 \V{R}_2^T \V{t}_2 + \V{t}_1 \\ \V{0}^T & 1
     \end{bmatrix}
   = \begin{bmatrix} \V{R}_{12} & \V{t}_{12} \\ \V{0}^T & 1 \end{bmatrix}
\]

### 3.2 Keypoint Matches Search

Keypoint matches are searched between the current keyframe \( KF_1 \) and 
a neighbor keyframe \( KF_2 \), with a preciously computed fundamental matrix
\( \V{F}_{12} \) (\( KF_2 \) to \( KF_1 \)) and corresponding camera parameters
\( \V{K}_1 \V{T}_1 = \V{K}_1 [\V{R}_1 | \V{t}_1] \) and 
\( \V{K}_2 \V{T}_2 = \V{K}_2 [\V{R}_2 | \V{t}_2] \). 
Implemented in function `ORBmatcher::SearchForTriangulation()`{.cpp}, 
the procedure is as follows:

(1) Compute epipole \( \V{e}_2 \) in \( KF_2 \): 
    \( \V{e}_2 = \V{K}_2 [\V{R}_2 | \V{t}_2] \V{X}_{cam1} \)
    where 
    \( \V{X}_{cam1} 
       = \V{T}_1^{-1} \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix}^T \)
    is the camera center of \( KF_1 \);
(2) For each keypoint in \( KF_1 \) that is not a map point yet,
    a) Find matched keypoints in \( KF_2 \) with the same node ID
       within bag-of-words model for feature matching acceleration;
    b) Find the best match in \( KF_2 \) from a) with the following criteria:
       - The keypoint is not a map point;
       - The keypoint is not already matched;
       - The Hamming distance between the descriptors of the 2 keypoints
         is below a preset threshold (50 in actual implementation);
       - The following equation is met:
         \( \|\V{e}_2 - \V{x}_2\|^2 \ge 100 s^o \)
         where \( \V{x}_2 \) is the keypoint coordinate in image, \( s \) is 
         ORB scale factor for its image pyramid, and \( o \) is the octave
         of the keypoint start from 0;
         - I think the equation means that the keypoint should not be near
           the epipole, and when this is the case, the actual 3D world point
           may be too near to the baseline of the 2 cameras, whose depth
           is too small;
         - PS: maybe the correct equation is 
           \( \|\V{e}_2 - \V{x}_2\| \ge 10 s^o \).
       - The distance from the keypoint to the corresponding epipolar line
         in the image of \( KF_2 \) should be smaller than a threshold:
         \( \frac {\V{l}_2^T \V{x}_2} {\sqrt{a^2 + b^2}} < 1.8 s^o \)
         where \( \V{l}_2 = \begin{bmatrix} a & b & c \end{bmatrix}^T \);
       - The orientation of the keypoint should be within the 3 most
         significant orientations (see [here](#map_init_mono_orbfm) 
         for more details).
         
### 3.3 Triangulation of New Map Points

New map points are triangulated from the foregoing searched keypoint matches 
between 2 views of current and a neighbor keyframe.

The procedure is as follows: for each keypoint match searched from the 
foregoing section,

(1) Check parallax between rays;
(2) If the parallax is appropriate, use direct linear transformation (DLT) 
    and then SVD to triangulate a 3D point;
(3) Check if the triangulated point is in front of both cameras;
(4) Check if the reprojection error is appropriate for the triangulated point
    in both cameras;
(5) If the triangulate point meets all the above restrictions, 
    add it to the map.
    
#### 3.3.1 Parallax Checking

The goal is to make sure that the parallax for observing the world point,
triangulated from the keypoint match between 2 keyframes,
is not too low. That is, the angle between 2 rays that is emitted from the
camera center to the world point is not near 0 degree.

First, compute the 3D coordinate of the keypoint with its origin 
at the camera center in both cameras corresponding to the 2 keyframes 
\( KF_1 \) and \( KF_2 \), assuming its depth is 1 (\( Z_{c1} = Z_{c2} = 1 \)):
\[
   \V{x}_1 
   = \begin{bmatrix} x_1 \\ y_1 \end{bmatrix}
   = \frac{1}{Z_{c1}} \V{K}_1 \V{X}_{c1}
   = \begin{bmatrix} f_{x1} & 0 & c_{x1} \\ 0 & f_{y1} & c_{y1} \end{bmatrix}
     \begin{bmatrix} X_{c1} \\ Y_{c1} \\ 1 \end{bmatrix}
\]
Therefore,
\[ 
   X_{c1} = \frac {x_1 - c_{x1}} {f_{x1}}, \quad
   Y_{c1} = \frac {y_1 - c_{x1}} {f_{y1}}. 
\]

The same is applied to the keypoint \( \V{x}_2 \) in \( KF_2 \):
\[ 
   X_{c2} = \frac {x_2 - c_{x2}} {f_{x2}}, \quad
   Y_{c2} = \frac {y_2 - c_{x2}} {f_{y2}}. 
\]

The ray for the view in \( KF_1 \) is computed as follows:
\begin{align*} 
  \V{r}_1 &= \V{R}_1^T \V{X}_{c1} \\
          &= \V{R}_1^T (\V{R}_1 \V{X}_{w1} + \V{t}_1) \\
          &= \V{X}_{w1} - (-\V{R}_1^T \V{t}_1) \\
          &= \V{X}_{w1} - \V{O}_{c1}
\end{align*}
where \( \V{X}_{w1} \) is the 3D world coordinate of the keypoint based on
the camera coordinate with a depth of 1, \( \V{O}_{c1} \) is the camera center
in the world coordinate system.

The same applied to the ray for the view in \( KF_2 \):
\[ \V{r}_2 = \V{X}_{w2} - \V{O}_{c2} \]

Then, we check the angle between the 2 rays as the angle of the parallax
between the 2 views:
\[ \theta = \arccos \frac {\V{r}_1 \cdot \V{r}_2} {\|\V{r}_1\| \|\V{r}_2\|} \]

The keypoint match can be used to triangulate new map point if
\(\theta \in (0, 90)\) degrees and \( \cos\theta < 0.9998 \) for
actual implementation. That is, the angle must not be too small to be 
near 0 degree.

#### 3.3.2 Triangulation

Given the 3D camera coordinates \( \V{X}_{c1} = (X_{c1}, Y_{c1}, 1)^T \) and
\( \V{X}_{c2} = (X_{c2}, Y_{c2}, 1)^T \) with depths to be 1,
based on each matched keypoint pair,
and the corresponding \( 3 \times 4 \) world to camera transformation matrices
\( \V{T}_1 \) and \( \V{T}_2 \), the triangulation process is based on the
equations below:
\[ \V{X}_{c1} = \V{T}_1 \V{X}_w, \quad \V{X}_{c2} = \V{T}_2 \V{X}_w \]
where \( \V{X}_w \) is the triangulated 3D world point.

From the equation for camera 1 from \( KF_1 \), we can use cross-product
to rewrite the equation as follows:
\[ \V{X}_{c1} \times \V{T}_1 \V{X}_w = \V{0} \]
\[
   [\V{X}_{c1}]_{\times} 
   \begin{bmatrix}
     \text{------} \V{t}_{11}^T \text{------} \\
     \text{------} \V{t}_{12}^T \text{------} \\
     \text{------} \V{t}_{13}^T \text{------} \\
   \end{bmatrix}
   \V{X}_w
   = \V{0}_{3 \times 1}
\]
\[
   \begin{bmatrix}
     0 & -1 & Y_{c1} \\ 1 & 0 & -X_{c1} \\ -Y_{c1} & X_{c1} & 0
   \end{bmatrix}
   \begin{bmatrix}
     \V{t}_{11}^T \V{X}_w \\
     \V{t}_{12}^T \V{X}_w \\
     \V{t}_{13}^T \V{X}_w \\
   \end{bmatrix}
   = \V{0}_{3 \times 1}
\]
\[
   \begin{bmatrix}
     - \V{t}_{12}^T \V{X}_w + Y_{c1} \V{t}_{13}^T \V{X}_w \\
       \V{t}_{11}^T \V{X}_w - X_{c1} \V{t}_{13}^T \V{X}_w \\
     *
   \end{bmatrix}
   = \V{0}_{3 \times 1}
\]
\[
   \begin{bmatrix}
     - \V{t}_{12}^T + Y_{c1} \V{t}_{13}^T \\
       \V{t}_{11}^T - X_{c1} \V{t}_{13}^T \\
     *
   \end{bmatrix} \V{X}_w
   = \V{0}_{3 \times 1}
\]

As the 3rd row of the left-hand-side matrix is just a linear combination of the 
1st and 2nd row, we discard the row, and then stack the similar 2 equations
from camera 2, and change the row order. Then we get the following:
\[
   \V{A} \V{X}_w
   = \begin{bmatrix}
       X_{c1} \V{t}_{13}^T - \V{t}_{11}^T \\
       Y_{c1} \V{t}_{13}^T - \V{t}_{12}^T \\
       X_{c2} \V{t}_{23}^T - \V{t}_{21}^T \\
       Y_{c2} \V{t}_{23}^T - \V{t}_{22}^T
     \end{bmatrix} \V{X}_w
   = \V{0}_{4 \times 1}
\]
where \( \V{t}_{ij}^T \) is the \( j \)th row of \( \V{T}_i \).

We then perform SVD decomposition on \( \V{A} \):
\begin{align*}
  \V{A}_{4 \times 4}
  &= \V{U} \BG{\Sigma} \V{V}^T \\
  &= \V{U} 
     \begin{bmatrix} 
       \sigma_1 & 0 & 0 & 0 \\ 0 & \sigma_2 & 0 & 0 \\ 0 & 0 & \sigma_3 & 0 \\
       0 & 0 & 0 & \sigma_4
     \end{bmatrix}
     \begin{bmatrix}
       \text{------} \V{v}_1^T \text{------} \\
       \text{------} \V{v}_2^T \text{------} \\
       \text{------} \V{v}_3^T \text{------} \\
       \text{------} \V{v}_4^T \text{------} \\
     \end{bmatrix}
\end{align*}
where \( \sigma_1 \ge \sigma_2 \ge \sigma_3 \ge \sigma_4 \).

The triangulated 3D world point in a homogeneous coordinate is then
assigned as \( \V{v}_4 \), the 4th column vector of the right-singular matrix 
corresponding with the smallest singular value \( \sigma_4 \). Ideally, as
\( \V{X}_w \) is mapped to the nullspace of \( \V{A} \), the smallest singular 
value \( \sigma_4 \) should be 0:
\[ \V{A} \V{v}_4 = \sigma_4 \V{v}_4 = \V{0}. \]

After the triangulated result is computed, check if the 4th dimension of 
\( \V{v}_4 = (v_1, v_2, v_3, v_4)^T \) is 0. 
Discard the triangulated result if \( v_4 = 0 \).

Finally, we normalize the triangulated result to have a scale factor of 1:
\[ \V{X}_w = (\frac{v_1}{v_4}, \frac{v_2}{v_4}, \frac{v_3}{v_4}, 1)^T \]

In actual implementation, the inhomogeneous coordinate of map points are stored.

#### 3.3.3 Post-check on Triangulated Points {#pc-tp}

The triangulated points should meet the following restrictions:

(1) They should be in front of both cameras:
    \( Z_c > 0 \) where \( Z_c \) is 3rd dimension of 
    \( \V{X}_c = \V{R} \V{X}_w + \V{t} \);
(2) Their reprojection errors in both camera should be below some thresholds;
    \[
       \|\V{x} - \frac{1}{Z_c} \V{K} (\V{R} \V{X}_w + \V{t})\|
       \le \chi \sigma
    \]
    where 
    - \( \V{x} \) is the 2D image coordinate of the keypoint;
    - \( Z_c \) is the Z-coordinate of the 3D camera coordinate;
    - \( \V{K} \) is the camera intrinsics;
    - \( \V{R} \) and \( \V{t} \) are the camera extrinsics;
    - \( \V{X}_w \) is the triangulated point; 
    - \( \chi^2 = 5.991 \) is an outlier threshold based on 
      \( \chi^2 \) test assuming the standard deviation of measurement error 
      is 1 pixel for homography;
    - \( \sigma = s^o \) where \( s \) is the scale factor of the ORB 
      image pyramid, \( o \) is the octave where the keypoint \( \V{x} \) is
      observed, starting from 0;
    - The above restriction is applied to both cameras in both keyframes.
(3) They should have scale consistency under the 2 cameras.
    - The scale consistency check is implemented as follows:

      ~~~{.cpp}
      // the ray emits from camera center of KF1/KF2 to the triangulated point
      cv::Mat normal1 = x3D-Ow1; 
      float dist1 = cv::norm(normal1); // ||d||
      cv::Mat normal2 = x3D-Ow2;
      float dist2 = cv::norm(normal2);
  
      if(dist1==0 || dist2==0) continue;
      
      const float ratioDist = dist2/dist1;
      const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]
                                / pKF2->mvScaleFactors[kp2.octave];
  
      // scale consistency check (ratioFactor = 1.5 * s^o)
      if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
          continue;
      ~~~
      
    - The equivalent restriction as an equation:
      \[
         \frac {1} {1.5 s^{o1}}
         \le
         \frac {s^{o_2} \cdot \|\V{X}_w - \V{O}_{c2}\|} 
               {s^{o_1} \cdot \|\V{X}_w - \V{O}_{c1}\|}
         = 
         \frac {\|\V{X}_w - \V{O}_{c2}\|_{max}} 
               {\|\V{X}_w - \V{O}_{c1}\|_{max}}
         \le
         1.5 s^{o1}
      \]
      where \( s \) is the ORB scale factor of the image pyramid,
      \( o1 \) and \( o2 \) are the octave where the keypoint is observed
      in camera 1 and 2 of \( KF_1 \) and \( KF_2 \); \( \V{X}_w \) is the
      triangulated 3D world point; \( \V{O}_{c1} \) and \( \V{O}_{c2} \)
      are the camera center of camera 1 and camera 2 in world coordinate
      system. \( \| \cdot \|_{max} \) is the max distance that is described
      [before](#imc).
    - Scale consistency: the distance from the triangulated point to the camera
      center should be consistent with the scale of the observed keypoint.
      The higher octave a keypoint is observed, the larger the range 
      of the distance is.


## 4. Map Point Searching and Fusing in Neighbor Keyframes

If currently no more new keyframes are needed to be processed, find best
neighbor keyframes of current keyframe, and search for more map points
linked to each neighbor keyframe. Fuse the duplicate map points for
each neighbor keyframe.

The procedure is as follows: for the current keyframe,

(1) Get the best neighbor keyframes (20 in actual implementation) from
    the covisibility graph, and for each neighbor keyframe, get
    its best neighbor keyframes (5 in actual implementation);
(2) Remove redundant and bad keyframes from (1), and the rest keyframes
    are the target keyframes for new map point searching and fusing scheme;
(3) Get all map points created from the current keyframe;
(4) Search for new keypoint matches and fuse duplicate ones for each target
    keyframe, based on map point info of current keyframe;
(5) For each target keyframes, get all map points created from the keyframe,
    and set the non-duplicated and non-bad ones as the candidate map points
    to be fused into the current keyframe;
(6) Search for new keypoint matches and fuse duplicate ones for current 
    keyframe, based on the map point info of the candidate map points 
    from all target keyframes;
(7) Update map point info (distinctive descriptor, normal, depth, ...) in 
    the current keyframe;
(8) Update connections in the covisibility graph based on the current keyframe.

### 4.1 Map Point Data Update and Fusion

Given an input keyframe and an input list of map points to be updated, 
the map point data in the input keyframe is updated, with fused/replaced better
map points, and newly added map points. Also, the input list of map points 
may also be updated.

The data above is updated via a feature matching scheme by projecting
input map points into image coordinate using camera instrinsics \( \V{K} \) and 
extrinsics \( \V{T} = [\V{R} | \V{t}] \) corresponding to the input keyframe.

The procedure, implemented in `ORBmatcher::Fuse()`{.cpp}, is as follows:
for each map point in the input list of map points,

(1) Project the map point from 3D world coordinate to 2D image coordinate
    by \( \V{x} = \frac{1}{Z_c} \V{K} (\V{R} \V{X}_w + \V{t}) \) where
    \( Z_c \) is the z-axis of camera coordinate 
    \( \V{X}_c = \V{R} \V{X}_w + \V{t} \). 
    Discard the whole procedure and continue processing the next map point 
    in the list if the following conditions are met:
    a) The camera coordinate \( \V{X}_c \) has a negative depth, i.e., 
       \( Z_c < 0 \);
    b) The projected \( \V{x} \) is out of the image border;
    c) The distance \( d_x \) from the camera center \( \V{O}_c \) to the map 
       point \( \V{X}_w \) is out of the distance range described 
       [here](#imc);
    d) The angle between the *normal* of the map point (which is described
       [here](#imc)) and the ray \( \V{r} = \V{X}_w - \V{O}_c \) is 
       at least 60 degrees.
(2) Search candidate keypoints around \( \V{x} \):
    a) Predict an appropriate octave \( o_x \) as if it were a keypoint observed
       by the camera corresponding to the input keyframe: given the scale 
       \( s \) of ORB image pyramid, the octave \( o \) of the keypoint which
       generates the map point, and the distance \( d \) from
       the center of the camera of the input keyframe, to the world
       position of the map point, the predicted octave is computed as follows:
       \[
          o_x
          = \lceil \log_s^{\frac{d \cdot s^o}{d_x}} \rceil
          = \lceil o + \log_s^{\frac{d}{d_x}} \rceil
       \]
       and \( o_x \in [0, o_{max}) \) where the ORB image pyramid has a maximum
       of \( o_{max} \) octaves;
    b) Compute search radius \( r = th \cdot s^{o_x} \), where \( th \) is a 
       parameter for controlling the radius. 
       \( th = 3 \) in actual implementation;
    c) Search and get all candidate keypoints around \( \V{x} \) with the radius
       of \( r \). If no candidate keypoint is found, discard the whole procedure
       and continue processing the next map point in the list.
(3) Find a best matching keypoint based on the projected \( \V{x} \) among the
    candidate keypoints: 
    a) Only find a best match among candidates whose octave 
       \( o_m \in [o_x - 1, o_x + 1] \) ("m" in the subscript means "matched");
    b) Compute the distance between \( \V{x} \) and the candidate \( \V{x}_m \),
       and discard current candidate if the distance is too large:
       \[ \frac {\|\V{x} - \V{x}_m\|} {s^{o_m}} > \chi \]
       where \( \chi \) is described [here](#pc-tp);
    c) Compute the Hamming distance between the ORB descriptors corresponding to
       \( \V{x} \) and \( \V{x}_m \);
    d) A best match is the one with minimum Hamming distance.
(4) Update map point data of the map point itself and that in the input keyframe
    if the Hamming distance between ORB descriptor of \( \V{x} \) and that of 
    the best match \( \V{x}_m \) is lower than a threshold 
    (50 in actual implementation):
    a) Get the corresponding map point \( \V{X}_m \) of the best matched keypoint 
       \( \V{x}_m \);
    b) If a corresponding map point is found, compare the number of 
       observations of \( \V{X}_w \) and \( \V{X}_m \),
       and replace the map point data of the map point having less observations
       with that of another map point;
       - The number of observations of a map point is the number of cameras that
         observe the keypoint corresponding to the map point, or the number 
         of keyframes that include the above keypoint data;
    c) If no map point is found, add an observation of the map point by the 
       input keyframe, and add the map point data into the keyframe.


## 5. Local Redundant Keyframe Removal {#lmkeyremove}

After *local* bundle adjustment is performed, redundant local keyframes
are removed from the map, in order to make map information as compact as 
possible.

The procedure, implemented in `LocalMapping::KeyFrameCulling()`{.cpp},
is as follows: for current keyframe,

(1) Get all keyframes connected with the current keyframe in the 
    covisibility graph;
(2) For each connected keyframe,
    a) Count the total number of non-bad map points in the keyframe,
       denoted as \( n_t \);
    b) Count the number of distinct map points that are observed by
       at least 3 other keyframes at the same or finer scale (lower octave)
       in the ORB image pyramid, denoted as \( n \);
    c) Remove the keyframe from the map if \( \frac {n} {n_t} > 0.9 \).
    
Note: keyframes can only be removed by this procedure, and all the other
      procedures in loop closing thread that try to remove them can just 
      trigger the removal if the criteria above is met.

### 5.1 Keyframe Removal

The removal of a keyframe from the map is implemented in function 
`KeyFrame::SetBadFlag()`{.cpp}.

The keyframe will not be immediately removed from the map. Only when loop edges
are empty, and the keyframe is flagged as "to be erased", the keyframe will 
then be removed.

The procedure to remove a keyframe is as follows:

(1) Remove connections from all keyframes that are connected to the current 
    keyframe;
(2) Remove connections from all map points linked to the current keyframe,
    that is, remove all observations from the current keyframe;
(3) Update the spanning tree of the keyframes without the node of 
    the current keyframe;
(4) Compute the relative pose from the parent of this keyframe to this keyframe:
    \( \V{T}_{cp} = \V{T}_{cw} \V{T}_{pw}^{-1} \) where \( \V{T}_{cw} \) and
    \( \V{T}_{pw} \) are the poses of current and its parent keyframe, 
    respectively ("*c*" stands for current 
    keyframe, "*p*" for parent of current frame, and "*w*" for world);
(5) Set bad flag on this keyframe, and remove this keyframe from the map and
    the keyframe database.

#### 5.1.1 Spanning Tree Update

If the current keyframe is to be removed, the children of the current keyframe
in the spanning tree have to be assigned a different parent node. 

The procedure is as follows: 

(1) Add the parent of current keyframe into the set of parent candidates;
(2) While the set of children nodes of the current keyframe is not empty, 
    for each iteration,
    a) Traverse each child keyframe, get its connected keyframes in the 
       covisibility graph, and for each its connected keyframe that is also 
       in the parent candidates set,
       check the edge weight between the child and the connected keyframe,
       and find the connected keyframe with the largest weight;
    b) The child with the largest edge weight between it and the connected 
       keyframe, which is in the parent candidates set, is assigned this
       connected keyframe as its parent;
    c) The child assigned a new parent in this iteration is removed from
       the children set of the current keyframe, and is added to the set
       of parent candidates;
    d) If there're children that have no link with any keyframes in the
       parent candidates set, meanwhile all other children are assigned
       a new parent, proceed to the next step.
(3) If the set of children nodes of the current keyframe is still not empty,
    assign each child in the set the parent of the current keyframe as
    their parent.


\newpage

