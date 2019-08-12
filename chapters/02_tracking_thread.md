# II. Tracking Thread

This Section gives detail on some specific procedures executed on the 
tracking thread.


## 1. Map Initialization (Monocular)
- Called by function `Tracking::MonocularInitialization()`{.cpp} if map is 
  not initialized
  
### 1.1 ORB Feature Matching {#map_init_mono_orbfm}
- For map initialization, the matching scheme is implemented by function  
  `ORBmatcher::SearchForInitialization()`{.cpp}
  
  ~~~{.cpp}
  // Matching for the Map Initialization (only used in the monocular case)
  int SearchForInitialization(
      Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, 
      std::vector<int> &vnMatches12, int windowSize=10);
  ~~~
  
- For each keypoint in initial frame:
  a) Get all keypoints within its neighborhood in current frame
  
     ~~~{.cpp}
     // F1: initial frame; F2: current frame
     for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++) {
         cv::KeyPoint kp1 = F1.mvKeysUn[i1];
         int level1 = kp1.octave;
         if(level1>0) continue;
         // neighborhood size is determined by windowSize
         vector<size_t> vIndices2 = F2.GetFeaturesInArea(
             vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);
     }
     ~~~
  
  b) Find best matching keypoint with minimum Hamming distance
  
     ~~~{.cpp}
     for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++) {
         // i1-th descriptor from initial frame
         cv::Mat d1 = F1.mDescriptors.row(i1);
         for(vector<size_t>::iterator vit=vIndices2.begin(); 
             vit!=vIndices2.end(); vit++) {
             size_t i2 = *vit;
             // i2-th descriptor from current frame
             cv::Mat d2 = F2.mDescriptors.row(i2);
             // compute Hamming distance between 2 descriptors
             int dist = DescriptorDistance(d1,d2);
         }
     }
     ~~~
  
  c) Screen matched keypoint using distance threshold and 
     feature orientation info
     
     ~~~{.cpp}
     // compute 3 most significant orientations for all matched keypoints
     ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
     for(int i=0; i<HISTO_LENGTH; i++) {
         // only keep matched keypoints within 3 most significant orientations
         if(i==ind1 || i==ind2 || i==ind3) continue;
         for(size_t j=0, jend=rotHist[i].size(); j<jend; j++) {
             int idx1 = rotHist[i][j];
             if(vnMatches12[idx1]>=0) {
                 vnMatches12[idx1]=-1;
                 nmatches--; // remove matched keypoints from other orientations
             }
         }
     }
     ~~~

### 1.2 Parallel Computation of 2 Models
- Motivation for computing 2 models: compute pose for different scenes
  - Homography \( \V{H} \): describe a planar scene better
  - Fundamental matrix \( \V{F} \): describe a non-planar scene better
- Implemented inside function `Initializer::Initialize()`{.cpp}, 
  which is called by function  
  `Tracking::MonocularInitialization()`{.cpp}
  
  ~~~{.cpp}
  // Launch threads to compute in parallel a fundamental matrix and a homography
  vector<bool> vbMatchesInliersH, vbMatchesInliersF;
  float SH, SF; // model score for H and F
  cv::Mat H, F; // homography & fundamental matrix
  thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), 
                 ref(SH), ref(H));
  thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), 
                 ref(SF), ref(F));
  // Wait until both threads have finished
  threadH.join();
  threadF.join();
  ~~~
  
- Homography \( \V{H} \) computation by normalized DLT [@Hartley2004]  
  ![norm_dlt](images/ch02/normalized_dlt.jpg){ width=80% }
  - Implemented in function `Initializer::FindHomography()`{.cpp}
- Fundamental matrix \( \V{F} \) computation by normalized 8-point algorithm  
  ![norm_8pt](images/ch02/normalized_8-point.jpg){ width=80% }
  - Implemented in function `Initializer::FindFundamental()`{.cpp}
  
### 1.3 Reconstruction of 3D Points and Camera Pose
- Inside function `Initializer::Initialize()`{.cpp}, 
  which is called by function  
  `Tracking::MonocularInitialization()`{.cpp}
- Using either homography \( \V{H} \) or fundamental matrix \( \V{F} \)
  for the reconstruction based on their model score **SH** and **SF**:

  ~~~{.cpp}
  // Compute ratio of scores
  float RH = SH/(SH+SF);
  // Try to reconstruct from homography or fundamental depending on the 
  // ratio (0.40-0.45)
  if(RH>0.40)
      return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,
                          vbTriangulated,1.0,50);
  else //if(pF_HF>0.6)
      return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,
                          vbTriangulated,1.0,50);
  ~~~

#### 1.3.1 Reconstruction Using Homography \( \V{H} \) {#reconstructH}
- Compute 8 possible pose solutions from \( \V{H} \) (implemented the method 
  proposed in [@Faugeras1988])
  - Let \( \V{A} = \V{K}^{-1} \V{H} \V{K} \) where \( \V{K} \) is the camera 
    calibration matrix
    - \( \V{H} \): homography between 2 2D pixel coordinates
    - \( \V{A} \): similar with \( \V{H} \) (share same eigenvalues),
      and denote homography between 2 3D camera coordinates
  - Assume \( \V{A} = d \V{R} + \V{t} \V{n}^T \) where 
    - Pose \( \V{P} = [\V{R} | \V{t}] \) (rotation & translation)
    - \( \V{n} \) is the normal to the image plane of reference frame
    - \( d = \V{n} \cdot \V{X}_1 \) where \( \V{X}_1 \) is a 3D point on the 
      image plane of reference frame, and \( d \) denotes the distance between 
      the plane and the origin
  - Do SVD on \( \V{A} \), and extract 8 possible solutions to 
    \( \{\V{R}, \V{t}\} \) based on the 3 singular values
- Select the best pose from the 8 hypotheses
  - Compute number of "good" keypoints for each pose (under function
    `Initializer::CheckRT()`{.cpp})
    - Triangulate all matched keypoints to get their 3D position w.r.t. 
      corresponding camera origin (via SVD)
      - \( \V{x} \times \V{P} \V{X} = \V{0} \to \V{A} \V{X} = \V{0} \)
      - Implemented in function `Initializer::Triangulate()`{.cpp}
        - \( \V{A} \) is constructed by 1 pair of matched keypoints and
          their corresponding camera poses (4 equations)
    - Get number of "good" keypoints for each pose
      - Criteria:
        - 3D space: low parallax and have positive depth for 
          triangulated 3D points
        - 2D space: low reprojection error for triangulated 3D points
  - Select the best pose with most number of "good" points or discard all
    poses if there's no clear winner

#### 1.3.2 Reconstruction Using Fundamental Matrix \( \V{F} \)
- Compute essential matrix \( \V{E}\): \( \V{E} = \V{K}^T \V{F} \V{K} \)
  where \( \V{K} \) is camera calibration matrix
- Compute 4 motion (pose) hypotheses: 
  (by calling function `Initializer::DecomposeE()`{.cpp})  
  - Assume pose for initial frame (from 1st frame to initial frame) is 
    \( \V{P}_{ini|1} = [\V{I}_{3 \times 3} | \V{0}_{3 \times 1}] \)
  - Assume SVD of essential matrix 
    \( \V{E} = \V{U} diag(1, 1, 0) \V{V}^T 
             = \begin{bmatrix} \V{u}_1 & \V{u}_2 & \V{u}_3 \end{bmatrix}
               diag(1, 1, 0) \V{V}^T \)
  - Pose of current frame can be:  
    \( \V{P}_{cur|1} = [ \V{U} \V{W} \V{V}^T | +\V{u}_3 ] \) or
    \( [ \V{U} \V{W} \V{V}^T | -\V{u}_3 ] \) or
    \( [ \V{U} \V{W}^T \V{V}^T | +\V{u}_3 ] \) or
    \( [ \V{U} \V{W}^T \V{V}^T | -\V{u}_3 ] \)  
    where \( \V{W} = 
    \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} \)
- Select the best pose from the 4 hypotheses: same as the selection procedure 
  from [previous section](#reconstructH)

### 1.4 Initial Map Creation {#imc}
- Set both initial and current frame as keyframes
- Compute bag-of-words for feature descriptors of the 2 keyframes
- Insert these 2 keyframes into the map
- Assign computed 3D keypoints as map points
  - Normal of a map point is computed in 
    `MapPoint::UpdateNormalAndDepth()`{.cpp}: 
    a 3x1 vector with the "average" direction from corresponding 
    camera centers (the cameras that observed the current map point) 
    to current 3D world position
    \[ 
       \V{n} = \frac{1}{k} \displaystyle\sum_{i=1}^k 
               \frac {\V{X}_w - \V{O}_{c, i}} {\|\V{X}_w - \V{O}_{c, i}\|}
    \]
    where \( \V{n} \) is the normal vector, \( \V{X}_w \) is the current 3D
    robot world position, and \( \V{O}_{c, i} \) is the \( i \)th camera 
    center in the world that observed the current map point
  - Distance range of a map point: also computed in 
    `MapPoint::UpdateNormalAndDepth()`{.cpp}, the distance range is 
    \( [\|\V{X}_w - \V{O}_c\| \cdot s^{o - o_{max}}, 
        \|\V{X}_w - \V{O}_c\| \cdot s^o] \)
    where \( \V{X}_w \) is the 3D world position of the map point, \( \V{O}_c \)
    is the camera center in the world coordinate system; \( s \) is the scale
    factor of ORB image pyramid, \( o \) is the octave level where the keypoint
    corresponding to the map point is observed, and \( o_{max} \) is the maximum
    octave level with minimal resolution in the pyramid
  - Distinctive descriptor for the map point: implemented in function  
    `MapPoint::ComputeDistinctiveDescriptors()`{.cpp}  
    Assume the map point is observed by \( k \) keyframes, there will be \( k \)
    ORB descriptors available. Compute Hamming distance between any 2 of them,
    so there're \( k \times k \) distances. Regard the distances as a 
    \( k \times k \) matrix. For each row, sort the \( k \) distances in 
    ascending order, and then find the median distance of each row.
    Find the minimum median distance among the \( k \) rows, and assume it is
    the \( i \)th row. Then, the distinctive descriptor for the map point is
    assigned with the ORB descriptor associated with the \( i \)th keyframe
- Add observations of these 2 keyframes
- Update connections in covisibility graph for these 2 keyframes
- Execute *full*/*global* bundle adjustment (BA) on current map
- Compute scene median depth and check whether depth info is valid
- If depth info is invalid or there're not enough map points for current frame,
  reset all info (nothing being initialized) and proceed to next frame
- Set miscellaneous info on related threads and set map status as initialized


## 2. Tracking

### 2.1 Pose Estimation

#### 2.1.1 Pose Estimation Based on Constant Velocity Motion Model
- Procedures
  (1) Directly compute new pose with velocity data
  (2) Search for map point correspondences
  (3) Optimize pose via *motion-only BA* (implemented in function
      `Optimizer::PoseOptimization()`{.cpp})
  (4) Discard map point outliers
- Condition: 
  (1) Tracking for last frame is successful
  (2) Velocity is already computed
- Tracking is successful if enough matched keypoints are found
  
##### 2.1.1.1 New Pose Computation
(1) Update info for last frame
    - Set pose for last frame 
      \( \V{P}_{k-1|1} = \V{P}_{k-1|x} \V{P}_{x|1} \)
      where the last keyframe is at frame \( x \)
    - Create *visual odometry* map points if
      - Last frame is not a keyframe, or
      - The sensor is a non-monocular one, or
      - ORB-SLAM2 system works in tracking-only mode
(2) Set pose for current frame
    \( \V{P}_{k|1} = \V{v} \V{P}_{k-1|1} 
                   = \V{P}_{k-1|k-2} \V{P}_{k-1|1}
                   = \V{P}_{k|k-1} \V{P}_{k-1|1} \)
    - Constant velocity assumption: 
      \( \V{v} = \V{P}_{k-1|k-2} = \V{P}_{k|k-1} \)

##### 2.1.1.2 Map Correspondences Searching: ORB Feature Matching {#searchproj}

Implemented in function `ORBmatcher::SearchByProjection()`{.cpp}.

~~~{.cpp}
// Project MapPoints tracked in last frame into the current frame and 
// search matches. Used to track from previous frame (Tracking).
int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, 
                       const float th, const bool bMono);
~~~

The Searching is done once or twice: if number of matched keypoints is not
enough, there will be a 2nd searching with 2x window size. If still get less
matched keypoints than expected, this tracking scheme is failed.

- Some pose-related variables:
 
  ~~~{.cpp}
  // R_{k|1} & t_{k|1}
  const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
  const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
  // t in P_{1|k} = [R | t]: - R_{1|k} t_{k|1}
  const cv::Mat twc = -Rcw.t()*tcw;
  // R_{k-1|1} & t_{k-1|1}
  const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
  const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);
  // tlc = R_{k-1|1} t_{1|k} + t_{k-1|1} = t_{k-1|1} - R_{k-1|k} t_{k|1}
  // (see illustration below)
  const cv::Mat tlc = Rlw*twc+tlw; 
  ~~~
  
  - Visual illustration for `cv::Mat tlc`{.cpp}:
    \begin{tikzpicture}
      % world origin
      \node [below left] at (0,0) {O};
      
      % robot position at time k-1 with pose P_{k-1|1}
      \draw [blue, thick] (30:3) circle [radius=0.2];
      \draw [blue, thick] (30:3.1) -- (30:3.4)
            node [below right, blue] {\( \V{P}_{k-1|1} \)};
      
      % translation vector at time k-1: t_{k-1|1}
      \draw [-stealth, ultra thick] (0,0) -- (30:3);
      \node [below right] at (30:1.5) {\( \text{tlw} = \V{t}_{k-1|1} \)};
      
      % robot position at time k with pose P_{k|1}
      \draw [blue, thick] (60:5) circle [radius=0.2];
      \draw [blue, thick] (60:5.1) -- (60:5.4)
            node [below right, blue] {\( \V{P}_{k|1} \)};
      
      % translation vector at time k: t_{k|1}
      \draw [-stealth, ultra thick] (0,0) -- (60:5);
      \node [left] at (60:3) {\( \text{tcw} = \V{t}_{k|1} \)};
      
      % rotation matrix from time k to time k-1: R_{k-1|k}
      \draw [->, dashed, thick] (60:5) to [out=-30, in=120] (30:5);
      \node [above right] at (45:5) {\( \V{R}_{k-1|k} \)};
      
      % rotated translation vector: - R_{k-1|k} t_{k|1}
      \draw [-stealth, green] (30:5) -- (0,0);
      \node [right, green] at (30:5) 
            {\( -\V{R}_{k-1|k} \V{t}_{k|1} \)};

      % tlc: t_{k-1|1} - R_{k-1|k} t_{k|1}
      \draw [-stealth, ultra thick, red, dashed] (30:5) -- (30:3);
      \node [below right, red] at (30:4) 
            {\( \text{tlc} = \V{t}_{k-1|1} - \V{R}_{k-1|k} \V{t}_{1|k} \)};
      
    \end{tikzpicture}

- For each keypoint in previous frame,
  a) Get candidate keypoints in current frame
     
     ~~~{.cpp}
     // Project 3D map point in world coordinate of frame k-1 
     // to camera coordinate using pose of current frame k
     cv::Mat x3Dw = pMP->GetWorldPos();
     cv::Mat x3Dc = Rcw*x3Dw+tcw; // world->cam coordinate conversion
     
     const float xc = x3Dc.at<float>(0);
     const float yc = x3Dc.at<float>(1);
     const float invzc = 1.0/x3Dc.at<float>(2);

     // depth must be positive (can be seen by camera)
     if(invzc<0)
         continue;

     // 3D cam coord -> 2D pixel coord projection
     float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
     float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

     // (u,v) must inside image border
     if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
         continue;
     if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
         continue;

     // octave of the ORB keypoint
     int nLastOctave = LastFrame.mvKeys[i].octave;

     // Search in a window. Size depends on scale & pre-defined threshold
     // (scale = base_scale ^ octave_num, num start from 0, 
     // base_scale = 1.2 by default)
     float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

     // candidate keypoints: within the grid cells covering 
     // [u-radius,u+radius]*[v-radius,v+radius] region
     // for octaves [nLastOctave-1,nLastOctave+1]
     vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, 
                                                nLastOctave+1);
     ~~~
     
     - Bag-of-words data stored in keyframe only, so here the candidate keypoints
       are searched in a window
     - Constant velocity: assume the keypoint position is determined by 
       relative pose (velocity)
   
  b) For each candidate keypoint in current frame,
     - Find best matching keypoint with minimum Hamming distance
     - Screen matched keypoint using distance threshold and 
       feature orientation info

#### 2.1.2 Pose Estimation Based on Reference Keyframe {#perefkf}
- Procedures
  (1) Do ORB feature matching to find map point correspondences
      a) Compute bag-of-words of ORB descriptors for current frame
      b) For each keypoint in reference keyframe, find matching keypoints in 
         current frame (implemented in function 
         `ORBmatcher::SearchByBoW()`{.cpp})
         
         ~~~{.cpp}
         // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
         // Brute force constrained to ORB that belong to the same vocabulary 
         // node (at a certain level). Used in Relocalisation and Loop Detection
         int SearchByBoW(KeyFrame *pKF, Frame &F, 
                         std::vector<MapPoint*> &vpMapPointMatches);
         ~~~
         
         - Use bag-of-words to get candidate keypoints
           - Only consider candidates of the **same node ID** for both keyframe
             and current frame
           - Advantage: more efficient than brute-force searching
         - Find best matching candidate keypoint with minimum Hamming distance
         - Screen matched keypoint using the same criteria described in 
           [ORB feature matching for map initialization](#map_init_mono_orbfm)
           part
  (2) Set pose of current frame the same as the last frame
  (3) Optimize pose via *motion-only BA*
- Conditions: 
  (1) No velocity is assigned
  (2) Robot is just relocalized after tracking is lost
  (3) Tracking is failed for last frame under constant velocity motion model
- Tracking is successful if enough matched keypoints are found

### 2.2 Global Relocalization
- Procedures
  (1) Compute bag-of-words of ORB descriptors for current frame
  (2) Find keyframe candidates from keyframe database for relocalization
      - Criteria for candidates: keyframe that shares enough words with current
        frame under bag-of-words representation
  (3) For each candidate keyframe,
      a) Search for keypoint correspondence using ORB bag-of-words as
         described [here](#perefkf) (implemented by function 
         `ORBmatcher::SearchByBoW()`{.cpp})
      b) Set up a PnP solver for the (keyframe, current frame) pair 
         if enough correspondences are found
  (4) For each PnP solver, find pose of current frame (\( \V{P}_{k|1} \)):
      a) Run a few iterations based on sets of randomly-chosen matched
         keypoints (RANSAC framework), and for each set of keypoints:
         - Compute the camera pose using [EPnP](https://icwww.epfl.ch/~lepetit/papers/lepetit_ijcv08.pdf) [@Lepetit2008] algorithm
         - Find the keypoints with sufficiently low reprojection error
           based on computed pose, and regard them as new inliers
         - If there are enough inliers, refine the pose using EPnP based on all
           available inliers
         - Update inlier info (number of inliers) based on refined pose
         - If there are enough inliers, return the refined pose
         - If there are not enough inliers, go to the next iteration
      b) After all the iterations, if there are enough inliers,
         return the current computed pose
      c) If there are not enough inliers, return an empty pose
  (5) For each candidate keyframe, optimize the computed pose 
      from step (4) using *motion-only BA* if the pose is not empty
      a) If there are too few keypoint correspondence inliers after 
         pose optimization, continue processing next candidate keyframe
      b) If there are not too few and still not enough keypoint correspondence 
         inliers after pose optimization, search for additional correspondences
         and optimize current pose using *motion-only BA* for at most 2 
         iterations
      c) If there are now enough keypoint correspondence inliers, record 
         current camera pose and stop further processing as the relocalization
         is already successful
  (6) Relocalization is failed if no qualified pose is computed

#### 2.2.1 Additional Keypoint Correspondence Searching {#searchproj2}
- Implemented in an overloaded version of function
  `ORBmatcher::SearchByProjection()`{.cpp}) by projecting map points
  seen in keyframe into current frame and matching between 2D image 
  coordinates of keyframe and current frame
  
  ~~~{.cpp}
  // Project MapPoints seen in KeyFrame into the Frame and search matches.
  // Used in relocalisation (Tracking)
  int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, 
                         const std::set<MapPoint*> &sAlreadyFound, 
                         const float th, const int ORBdist);
  ~~~
  
- Differences between this scheme and that described [above](#searchproj) 
  - Different search window
    
    ~~~{.cpp}
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    // Depth must be inside the scale pyramid of the image
    if(dist3D<minDistance || dist3D>maxDistance) continue;
    // have to predict the scale of the keypoint in the pyramid
    int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);
    // Search in a window
    const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];
    // candidate keypoints: within the grid cells covering 
    // [u-radius,u+radius]*[v-radius,v+radius] region
    // for octaves [nPredictedLevel-1,nPredictedLevel+1]
    const vector<size_t> vIndices2 = 
        CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, 
                                       nPredictedLevel+1);
    ~~~

### 2.3 Local Map Tracking
- Procedures
  (1) Update local map for current frame
      a) Update keyframe info of local map
      b) Update map point info of local map
  (2) Search for more keypoint correspondences in updated local map
  (3) Optimize pose \( \V{P}_{k|1} \) of current frame via *motion-only BA*
  (4) Check if tracking is successful based on the newly optimized pose
  
#### 2.3.1 Keyframe Info Update of Local Map
- Include keyframes by which at least one map point in 
  current frame is observed
- Include keyframes that are neighbors of previously included keyframes: for
  each previously included keyframe,
  (1) Include neighbors in the covisibility graph that are closest to current
      keyframe
      - 10 such neighbors for actual implementation
  (2) Include extra neighbors that are the childs of current keyframe in the 
      spanning tree generated from the covisibility graph
  (3) Include extra neighbors that are the parents of current keyframe in the 
      spanning tree generated from the covisibility graph
  (4) Stop including more keyframes if the number of them exceeds an
      upper limit (80 in actual implementation)
- Set keyframe having most shared keypoints with current frame as reference
  keyframe

#### 2.3.2 Keypoint Correspondence Searching in Local Map {#searchproj3}
- Implemented in another overloaded version of function 
  `ORBmatcher::SearchByProjection()`{.cpp})
  
  ~~~{.cpp}
  // Search matches between Frame keypoints and projected MapPoints. 
  // Returns number of matches. Used to track the local map (Tracking).
  int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, 
                         const float th=3);
  ~~~
  
- Differences between this scheme and the previous described schemes
  ([here](#searchproj) and [here](#searchproj2))
  - No need to compute projected 2D image coordinates given input map points
    in the local map (projected coordinates already computed by function
    `Frame::isInFrustum()`{.cpp})
  - Different search window
    
    ~~~{.cpp}
    // get predicted scale level
    const int &nPredictedLevel = pMP->mnTrackScaleLevel;
    // The size of the window will depend on the viewing direction
    float r = RadiusByViewingCos(pMP->mTrackViewCos);
    // multiply threshold if it is not 1.0
    if(bFactor) r*=th;
    // candidate keypoints: within the grid cells covering 
    // [u-radius,u+radius]*[v-radius,v+radius] region
    // for octaves [nPredictedLevel-1,nPredictedLevel]
    // where u = pMP->mTrackProjX, v = pMP->mTrackProjY, 
    // radius = r*F.mvScaleFactors[nPredictedLevel]
    const vector<size_t> vIndices =
        F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,
        r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);
    ~~~

### 2.4 Velocity Data Update

Update velocity data of current frame using the poses of current and last frame:

- Pose of last ((\( k-1 \))th) frame (from frame 1 to frame (\( k-1 \)))
  \( \V{P}_{k-1|1} =
     \begin{bmatrix} \V{R}_{k-1|1} & \V{t}_{k-1|1} \\ 0 & 1 \end{bmatrix} \)
  - Rotation matrix \( \V{R}_{k-1|1} \) & translation vector \( \V{t}_{k-1|1} \)
    are accumulated from 1st frame to (\( k-1 \))th frame
  - Its inverse 
    \( \V{P}_{k-1|1}^{-1} = \V{P}_{1|k-1} = 
       \begin{bmatrix} 
         \V{R}_{k-1|1}^T & -\V{R}_{k-1|1}^T \V{t}_{k-1|1} \\ 0 & 1
       \end{bmatrix} \)
- Pose of current \( k \)th frame from frame 1
  \( \V{P}_{k|1} = 
     \begin{bmatrix} \V{R}_{k|1} & \V{t}_{k|1} \\ 0 & 1 \end{bmatrix} \)
- Velocity:
  \begin{align*} 
    \V{V} &= \V{P}_{k|1} \V{P}_{k-1|1}^{-1} \\
          &= \begin{bmatrix}
               \V{R}_{k|1} \V{R}_{k-1|1}^T & 
               -\V{R}_{k|1} \V{R}_{k-1|1}^T \V{t}_{k-1|1} + \V{t}_{k|1} \\
               0 & 1
             \end{bmatrix} \\
          &= \begin{bmatrix}
               \V{R}_{k|1} \V{R}_{1|k-1} & 
               -\V{R}_{k|1} \V{R}_{1|k-1} \V{t}_{k-1|1} + \V{t}_{k|1} \\
               0 & 1
             \end{bmatrix} \\
          &= \begin{bmatrix}
               \V{R}_{k|k-1} & \V{t}_{k|1} - \V{R}_{k|k-1} \V{t}_{k-1|1} \\
               0 & 1
             \end{bmatrix} \\
          &= \V{P}_{k|1} \V{P}_{1|k-1} \\
          &= \V{P}_{k|k-1}
  \end{align*}
    - Relative pose from frame (\( k-1 \)) to frame \( k \)
    
  - Illustration:
    \begin{tikzpicture}
      % world origin
      \node [below left] at (0,0) {O};
      
      % robot position at time k-1
      \draw [blue, thick] (30:3) circle [radius=0.2];
      %\fill [blue] (30:3) circle [radius=0.05];
      \draw [blue, thick] (30:3.1) -- (30:3.4)
            node [below right, blue] {\( \V{P}_{k-1|1} \)};
      
      % translation vector at time k-1
      \draw [-stealth, ultra thick] (0,0) -- (30:3);
      \node [below right] at (30:1.5) {\( \V{t}_{k-1|1} \)};
      
      % robot position at time k
      \draw [blue, thick] (60:5) circle [radius=0.2];
      %\fill [blue] (60:5) circle [radius=0.05];
      \draw [blue, thick] (60:5.1) -- (60:5.4)
            node [below right, blue] {\( \V{P}_{k|1} \)};
      
      % translation vector at time k
      \draw [-stealth, ultra thick] (0,0) -- (60:5);
      \node [left] at (60:3) {\( \V{t}_{k|1} \)};
      
      % rotation matrix from time k-1 to time k
      \draw [->, dashed, thick] (30:3) to [out=120, in=-30] (60:3);
      \node [above right] at (45:3) {\( \V{R}_{k|k-1} \)};
      
      % translation vector of time k-1 rotated by rotation matrix 
      % from time k-1 to time k
      \draw [->, violet] (0,0) -- (60:3);
      \node [above left, violet] at (60:1.5) {\( \V{t}_{k-1|1, rotated} \)};
      
      % rotated pure translation vector from time k-1 to time k
      \draw [->, red] (60:3) -- (60:5);
      \node [above left, red] at (60:4) {\( \V{t}_{k|k-1, rotated} \)};
    \end{tikzpicture}
    where 
    - \( \V{t}_{k-1|1, rotated} = \V{R}_{k|k-1} \V{t}_{k-1|1} \)
    - \( \V{t}_{k|k-1, rotated} = \V{t}_{k|1} - \V{t}_{k-1|1, rotated} \)
    
- Implementation:

  ~~~{.cpp}
  // inverse pose for last frame (camera to world)
  // [R, t; 0, 1] -> [R^T, -R^T*t; 0, 1]
  cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
  mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
  mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
  // v = [R_{k|0}, t{k|0}; 0, 1] 
  //     * [R_{k-1|0}^T, -R_{k-1|0}^T * t_{k-1|0}; 0, 1]
  //   = [R_{k|k-1}, t_{k|0} - R_{k|k-1} * t_{k-1|0}; 0, 1]
  mVelocity = mCurrentFrame.mTcw*LastTwc;
  ~~~

### 2.5 New Keyframe Insertion
- Procedures
  (1) Check whether to add current frame into keyframe set
  (2) If current frame is needed to be added into the keyframe set,
      create a new keyframe using info of current frame
      - Set this new keyframe as reference keyframe to current frame

#### 2.5.1 Keyframe Insertion Check
- Related conditions (monocular only):
  (1) Local mapping is not freezed by a loop closure
  (2) Enough frames (fps in actual implementation) have passed from latest
      relocalization
  (3) Enough frames (fps in actual implementation) have passed from latest
      keyframe insertion
  (4) Enough frames (0 in actual implementation) have passed and 
      local mapping is idle
  (5) Number of tracked keypoints is large enough (15 in actual implementation) 
      but less than a portion (90% in actual implementation) of 
      reference keyframe
- Procedures
  - Reject insertion if any of the conditions (1), (2), (5) is not met, 
    or (3) and (4) are not met simultaneously
  - If keyframe insertion is currently not rejected, check if local mapping
    is idle
    - If local mapping is idle, insert a new keyframe
    - Otherwise, interrupt *local BA* running in local mapping thread
      for inserting future qualified keyframe as soon as possible,
      and then reject keyframe insertion

\newpage
