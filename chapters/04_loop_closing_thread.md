# IV. Loop Closing Thread

This section gives detail on specific procedures executed on this thread.


## 1. Loop Detection {#ld}

The procedure for detecting loop in the map is as follows: for the first
keyframe (oldest new keyframe) in the keyframe queue of the loop closing
thread,

(1) Add the keyframe into the keyframe database if there're less than \( k \)
    keyframes in the map, or if less than \( k \) keyframes are added in the
    map after a latest loop is detected (\( k = 10 \) in actual implementation),
    and continue processing the next incoming keyframe. Otherwise, go to the
    next step;
(2) Compute similarity scores between the bag-of-words vector of 
    current keyframe and that of each keyframe connected with the current
    keyframe in the covisibility graph, and maintain the lowest score
    among them as a baseline for loop detection;
(3) Find keyframes which are loop candidates based on the baseline score;
(4) If no loop candidates are found, try to erase the current keyframe,
    and add current keyframe to the keyframe database for subsequent
    loop detection. Also, clear the maintained list of loop candidate groups.
    Then continue processing the next incoming keyframe;
    - All operations that try to erase/remove the keyframe in this thread is 
      implemented in function `KeyFrame::SetErase()`{.cpp};
    - The keyframe removal operation is done only when a bad flag is set
      on the keyframe, and the flag will only be set by the local mapping thread
      when it tries to remove redundant keyframes (check [here](#lmkeyremove)
      for details);
    - Only the redundant keyframes detected by the local mapping thread
      will be removed. 
(5) If loop candidates are found, for each loop candidate keyframe,
    check the consistency between current and previously maintained loop 
    candidates that are found based on previous incoming keyframes.
    If there're enough consistency, add the current loop candidate into
    a list for loop candidates with enough consistency;
(6) Add the current keyframe into the keyframe database;
(7) If the loop candidate list with enough consistency is not empty,
    a loop is detected; otherwise no loop is detected.

### 1.1 Similarity Score Computation

The similarity score between 2 bag-of-words vector 
\( \V{a} = (a_1, \dots, a_n) \) and \( \V{b} = (b_1, \dots, b_n) \) 
is related to the \( l_1 \)-norm, and is computed as follows:
\[
   s_{l_1} 
   = - \frac{1}{2} 
     \displaystyle\sum_{i=1}^n \left( |a_i - b_i| - |a_i| - |b_i| \right)
\]
The larger the score, the more similar the 2 vectors are. 
\(s_{l_1} = \sum_i |a_i| = \|\V{a}\|_1 \) if \( \V{a} = \V{b} \), and
\(s_{l_1} = 0 \) if \( \V{a} = - \V{b} \).

The score computation scheme is determined by the constructor of 
the bag-of-words class `ORB_SLAM2::ORBVocabulary`{.cpp}, which has the 
actual type of  
`DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>`{.cpp}. 
The default constructor of this class is defined as follows:

~~~{.cpp}
DBoW2::TemplatedVocabulary(int k = 10, int L = 5, 
    WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);
~~~

### 1.2 Loop Candidates Detection

Loop candidates detection scheme based on an input keyframe is implemented 
in function `KeyFrameDatabase::DetectLoopCandidates()`{.cpp}.

The procedure is as follows:

(1) Search all keyframes that share a word with the input keyframe based
    on their bag-of-words representations, and only include the ones that
    is not connected with the input keyframe in the covisibility graph
    into a list of keyframe candidates;
(2) If no keyframe candidate is found in step 1, terminate the loop candidate 
    detection scheme, and no loop candidate is found;
(3) If there're keyframe candidates found in step 1, get the maximum number
    of shared words from the list of keyframe candidates, and denote it
    as \( n_{max} \);
(4) Compute similarity score between the input keyframe and keyframe
    candidates in the list whose number of shared words with the input keyframe
    is larger than \( 0.8 \cdot n_{max} \), and keep the candidates whose score
    is not less than the baseline score;
(5) If no keyframe candidates left after step 4, terminate the detection
    scheme, and no loop candidate is found;
(6) If there are still keyframe candidates left, for each keyframe candidate,
    a) Find their 10 best neighbor keyframes (connected keyframes with 10 
       largest edge weights);
    b) For each neighbor keyframe, if they have their similarity score computed,
       that is, if they are currently in the list of keyframe candidates, 
       accumulate the similarity score of this neighbor keyframe, and
       find the neighbor keyframe with the highest similarity score. Then
       the accumulated similarity score is bound to the neighbor keyframe
       with the highest score, which is one of the keyframe candidates.
(7) Record the best accumulative similarity score among the keyframe candidates
    in the last step, and denote is as \( s_{cb} \);
(8) For the keyframe candidates in step 6, assign the keyframes whose bound 
    accumulated similarity score \( s_c \ge 0.75 \cdot s_{cb} \) as loop 
    candidates.

### 1.3 Consistency Check for Loop Candidate

For each loop candidate keyframe, the consistency check is as follows:

(1) Get connected keyframes in the covisibility graph and group them and the
    loop candidate into a loop candidate group (set);
(2) The loop candidate group is compared with previously maintained
    list of loop candidate groups, and if the 2 groups have at least
    one common keyframe, the consistency index of the current loop candidate
    group will be added by 1.
(3) If there're \( k \) consecutive input keyframes processed by loop closing
    thread that have loop candidates found, and at least one of their 
    corresponding loop candidate groups have common keyframe with a certain
    one of the previously maintained list of loop candidate groups, 
    the consistency index for the latest loop candidate group will be \( k \),
    and if \( k \ge th \), which is a consistency threshold 
    (3 in actual implementation), the current loop candidate keyframe will
    be added into a list of loop candidate with enough consistency.
    - If only less than \( k \) consecutive input keyframes meet the
      conditions above, the consistency index will be reset to 0 because
      the maintained list of loop candidate groups will be cleared.
  

## 2. Similarity Transformation Computation

After a loop is detected, in this step, a similarity transformation 
\( \V{S}_{cl} = 
   \begin{bmatrix} s_{cl} \V{R}_{cl} & \V{t}_{cl} \\ \V{0}^T & 1 \end{bmatrix} 
   \in Sim(3) \) 
from each loop candidate keyframe to the current keyframe
found [above](#ld) is computed. Its inverse \( \V{S}_{cl}^{-1} = \V{S}_{lc} \) 
is also computed. 
They are noted as \( \V{S} \) if not specifically distinguished.

The procedure is as follows: 

(1) Do ORB feature matching between current keyframe and each loop candidate 
    keyframe by function `ORBmatcher::SearchByBoW()`{.cpp} (similar to
    the overloaded version described [here](#perefkf));
    
    ~~~{.cpp}
    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node 
    // (at a certain level), Used in Relocalisation and Loop Detection.
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, 
                    std::vector<MapPoint*> &vpMatches12);
    ~~~
    
(2) For each loop candidate, if the number of matches from step 1 is not 
    less than 20 (for actual implementation), set up a \( Sim(3) \) solver 
    based on data from current keyframe and the loop candidate keyframe, 
    in order to compute the similarity transformation \( \V{S} \).
    Otherwise continue processing the next loop candidate;
(3) For each \( Sim(3) \) solver based on a (current keyframe, loop candidate
    keyframe) pair, try to find a similarity transformation \( \V{S} \):
    a) Run a few iterations based on sets of randomly-chosen map point 
       correspondences (RANSAC framework), and for each set of map point
       correspondence:
       - Compute similarity transformation \( \V{S} \)
         based on 3 pairs of randomly-chosen map point correspondences;
       - Check if the pairs of map point are inliers after projected onto
         image plane using computed \( \V{S} \) and camera intrinsics 
         \( \V{K} \): 
         \[ \V{x}_{c,c} = \frac{1}{Z_{c,c}} \V{K}_c \V{S}_{cl} \V{X}_{w,l} \]
         where \( \V{x}_{c,c} \) is the 2D image coordinate from camera of
         current keyframe, \( \V{X}_{w,l} \) is the 3D world coordinate
         of the loop candidate keyframe; \( \V{K}_c \) is the camera intrinsics
         of current keyframe, and \( Z_{c,c} \) is the Z coordinate of
         the camera coordinate in current keyframe  mapped by the similarity 
         transformation \( \V{S}_{cl} \) from loop candidate keyframe 
         to current keyframe;
       - Record the best \( \V{S} \) with most inliers for each iteration;
       - If accumulated number of inliers exceeds a threshold (20 in actual 
         implementation), return the best \( \V{S} \) computed; otherwise,
         no \( \V{S} \) result is computed.
    b) After all the iterations, if no \( \V{S} \) is computed, continue
       running next \( Sim(3) \) solver;
    c) If \( \V{S} \) can be computed, find map point correspondences between
       current and the corresponding loop candidate;
    d) Optimize computed \( Sim(3) \) transformation \( \V{S} \) by previously
       found map point correspondences;
    e) If inliers are no less than a threshold (20 in actual implementation),
       record current \( \V{S} \) info and stop further processing as the 
       \( Sim(3) \) transformation computation is successful. The corresponding
       loop candidate keyframe becomes a loop keyframe.
(4) If \( \V{S} \) with enough map point inliers is not computed, remove
    all the loop candidates and the current keyframe from the map, and
    continue processing the next input keyframe in the loop closing thread;
(5) If \( \V{S} \) is computed successfully in step 3, retrieve all distinct 
    map points observed by the corresponding loop keyframe and all its neighbors
    in the covisibility graph;
(6) Find map point matches between the current keyframe and the loop keyframe,
    and if there are enough correspondences (40 in actual implementation),
    accept the detected loop and the corresponding loop keyframe. All the other
    loop candidate keyframes are removed from the map.

### 2.1 \( Sim(3) \) Computation based on 3 Pairs of Map Point Correspondences

In ORB-SLAM2 system, \( \V{S} \) is computed based on 3 pairs of map point 
correspondences in current keyframe and in a loop candidate keyframe, 
using the [@Horn1987] 
([paper](https://pdfs.semanticscholar.org/3120/a0e44d325c477397afcf94ea7f285a29684a.pdf)) method (see section 4.C for a summary of the method).

The computed \( \V{S} \) is a mapping from 3D world coordinate of one view
to the camera coordinate of another view. That is,
\[ 
   \V{X}_{c,c} = \V{S}_{cl} \V{X}_{w,l}, \quad 
   \V{X}_{c,l} = \V{S}_{lc} \V{X}_{w,c} 
\]
where
\( \V{X}_{w,c} \) and \( \V{X}_{w,l} \) are the world coordinates in
current and loop candidate keyframe, respectively;
\( \V{X}_{c,c} \) and \( \V{X}_{c,l} \) are the camera coordinates in
current and loop candidate keyframe, respectively.


The actual implementation is divided into the following steps:

(1) Compute the centroid and the point coordinates relative to the centroid
    (see section 2.C for details);

    Given 3 map points \( \{\V{X}_{c,1}, \V{X}_{c,2}, \V{X}_{c,3}\} \) 
    in the current keyframe, and 3 corresponding map points 
    \( \{\V{X}_{l,1}, \V{X}_{l,2}, \V{X}_{l,3}\} \) in the loop candidate 
    keyframe, compute their centroid \( \V{C}_* \)  as follows:

    \[ \V{C}_c = \frac{1}{3} \displaystyle\sum_{i=1}^3 \V{X}_{c,i}, \quad
       \V{C}_l = \frac{1}{3} \displaystyle\sum_{i=1}^3 \V{X}_{l,i} \]
    
    Let 
    \( \V{P}_c = 
       \begin{bmatrix} \V{X}_{c,1} & \V{X}_{c,2} & \V{X}_{c,3} \end{bmatrix} \), 
    and
    \( \V{P}_l = 
       \begin{bmatrix} \V{X}_{l,1} & \V{X}_{l,2} & \V{X}_{l,3} \end{bmatrix} \),
    the relative coordinate in the above matrix form is computed as follows:
    
    \[ \V{P}_c' = \V{P}_c - \V{C}_c, \quad \V{P}_l' = \V{P}_l - \V{C}_l \]

(2) Compute a \( 3 \times 3 \) matrix \( \V{M} \) (see section 4.A for details);

    \[ \V{M} = \V{P}_l' \V{P}_c'^T \]

(3) Compute a \( 4 \times 4 \) matrix \( \V{N} \) which is the product
    of 2 \( 4 \times 4 \) matrices that are expanded from 2 unit quaternions
    (see section 4 and 4.A for details);
    
    Let 
    \[ 
       \V{M} = \begin{bmatrix} 
                 S_{xx} & S_{xy} & S_{xz} \\
                 S_{yx} & S_{yy} & S_{yz} \\
                 S_{zx} & S_{zy} & S_{zz} 
               \end{bmatrix}
    \]
    Then
    \[
       \V{N} = \V{N}^T =
       \begin{bmatrix}
         S_{xx}+S_{yy}+S_{zz} & S_{yz}-S_{zy} & S_{zx}-S_{xz} & S_{xy}-S_{yx} \\
         * & S_{xx}-S_{yy}-S_{zz} & S_{xy}+S_{yz} & S_{zx}+S_{xz} \\
         * & * & -S_{xx}+S_{yy}-S_{zz} & S_{yz}+S_{zy} \\
         * & * & * & -S_{xx}-S_{yy}+S_{zz}
       \end{bmatrix}
    \]

(4) Compute the rotation element \( \V{R}_{cl} \) in \( \V{S}_{cl} \)
    (see section 4.B for details);

    Find the eigenvector \( \V{e}_{max} \) of \( \V{N} \) 
    with the highest eigenvalue \( \lambda_{max} \), 
    which is the unit quaternion representation 
    of the rotation element \( \V{R}_{cl} \) in the similarity transformation 
    \( \V{S}_{cl} \in Sim(3) \) from current to loop candidate keyframe. The
    conversion from the unit quaternion \( \V{e}_{max} \) to \( \V{R}_{cl} \)
    is trivial.
    
(5) Compute the scale element \( s_{cl} \) in \( \V{S}_{cl} \) 
    (see section 2.D and 2.E for details);

    The actual implementation in ORB-SLAM2 system uses the 2nd formula
    in section 2.E of the paper for the asymmetry case where the map point
    measurements in one of the views are known with much better precision than 
    those in the other view. The better view is assumed to be the view measured
    by the loop candidate keyframe.
    
    \[ 
       s_{cl}
       = \frac
         { \displaystyle\sum_{i=1}^3 
           \V{X}_{c,i}' \cdot (\V{R}_{cl} \V{X}_{l,i}') }
         { \displaystyle\sum_{i=1}^3 \| \V{R}_{cl} \V{X}_{l,i}' \|^2 }
       = \frac
         { \displaystyle\sum_{i=1}^3 
           \V{X}_{c,i}' \cdot (\V{R}_{cl} \V{X}_{l,i}') }
         { \displaystyle\sum_{i=1}^3 \| \V{X}_{l,i}' \|^2 }
    \]

(6) Compute the translation element \( \V{t}_{cl} \) in \( \V{S}_{cl} \)
    (see section 2.B and 2.C for details);

    \[ \V{t}_{cl} = \V{C}_c - s_{cl} \V{R}_{cl} \V{C}_l \]

(7) Combine the elements in \( \V{S}_{cl} \) together, and compute its
    inverse \( \V{S}_{cl}^{-1} = \V{S}_{lc} \).
    
    \[
       \V{S}_{cl}^{-1} = \V{S}_{lc} = 
       \begin{bmatrix}
         s_{cl}^{-1} \V{R}_{cl}^T & -s_{cl}^{-1} \V{R}_{cl}^T \V{t}_{cl} \\ 
         \V{0}^T & 1
       \end{bmatrix}
    \]


## 3. Loop Correction


\newpage
