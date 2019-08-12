# V. Bundle Adjustment


## 1. Overview

Bundle adjustment (BA) in ORB-SLAM2 is implemented using 
[g2o](https://github.com/RainerKuemmerle/g2o) [@Kummerle2011].
The operations on Lie group SE(3) and SO(3) is implemented by extending basic
classes defined in g2o.  

The followings are related materials:

- g2o: 
  [paper](https://www.cct.lsu.edu/~kzhang/papers/g2o.pdf), 
  [documentation](https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf)
- Lie group and related operations
  - [Lie Groups for 2D and 3D Transformations](http://ethaneade.com/lie.pdf) 
    by Ethan Eade
    - Related formulas are implemented in ORB-SLAM2
  - Chapter 7 on [@Barfoot2017] 
    ([link](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf))

### 1.1 Nonlinear Graph Optimization Using Least-Squares
- Optimization scheme:
  \[
     \V{x}^* = \argmin_{\V{x}} f(\V{x})
  \]
  where
  \[
     f(\V{x}) 
     = \displaystyle\sum_{i,j} 
       \underbrace{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{e}_{ij}}_{f_{ij}}
  \]
  - \( f(\cdot) = \sum_{i,j} f_{ij} \): \( 1 \times 1 \) scalar cost function
  - \( \V{e} \): error function with dimension \( nk \times 1 \) 
    (assuming there are \( n \) terms in \( f(\cdot) \))
  - Each \( \V{e}_{ij} \) term has the same \( nk \times 1 \)
    dimension
    - \( \V{e}_{ij} = \V{e}(\V{x}_i, \V{x}_j, \V{z}_{ij})
                    = \V{e}(\V{x}) \)
      - Vector error function that measures how well the parameter blocks
        (vertices in the g2o graph)
        \( \V{x}_i \) and \( \V{x}_j \) satisfy the constraint 
        \( \V{z}_{ij} \) (edges in the g2o graph)
      - \( \V{x} = (\V{x}_1, \dots, \V{x}_n)^T \): the vector to be 
        optimized with dimension \( nk \times 1 \)
        - Dimension of \( \V{x}_{i} \): \( k \times 1 \)
  - \( \BG{\Omega} \): information matrix (inverse of covariance matrix 
    \( \BG{\Sigma} \)) with dimension \( nk \times nk \)
- Iteration step: \( \V{x} \leftarrow \V{x} \oplus \Delta\V{x} \)
  - The operator "\( \oplus \)" is the "addition" operation in a smooth 
    manifold as the vector \( \V{x} \) may not be in an Euclidean space where
    the normal addition operator "\( + \)" is defined
    - For example, camera poses are in a \( SE(3) \) Lie group, where
      the equivalent "addition" operation is the multiplication operation
- Find the best \( \Delta\V{x} \) for each iteration:
  - 1st-order Taylor expansion around current \( \V{x}_{ij} \) 
    on error function \( \V{e}_{ij} \):
    \[
       \V{e}_{ij}(\V{x}_i \oplus \Delta\V{x}_i, \V{x}_j \oplus \Delta\V{x}_j)
       \approx \V{e}_{ij}(\V{x}) + \V{J}_{ij} \Delta\V{x}
       = \V{e}_{ij} + \V{J}_{ij} \Delta\V{x}
    \]
    where
    - \( \V{e}_{ij} = (\V{0}, \dots, \V{e}_i, \dots, \V{e}_j, \dots, \V{0})^T \)
      is a \( nk \times 1 \) vector with each \( \V{e}_i \) a \( k \times 1 \)
      vector
    - \( \V{x} \) in \( \V{e}_{ij} \) is a *sparse* vector:
      \( \V{x} = (\V{0}, \dots, \V{x}_i, \dots, \V{x}_j, \dots, \V{0})^T \)
    - \( \V{J}_{ij} = \frac{\partial \V{e}_{ij}}{\partial \V{x}} \)
      is the Jacobian of error function term \( \V{e}_{ij} \)
      w.r.t. parameter vector \( \V{x} \)
  - Expand \( f_{ij}(\V{x} \oplus \Delta\V{x}) \) using the above approximation
    (Gauss-Newton method):
    \begin{align*}
      f_{ij}(\V{x} \oplus \Delta\V{x})
      &= \V{e}_{ij}(\V{x} \oplus \Delta\V{x})^T \BG{\Omega}_{ij} 
         \V{e}_{ij}(\V{x} \oplus \Delta\V{x}) \\
      &= (\V{e}_{ij} + \V{J}_{ij} \Delta\V{x}_{ij})^T \BG{\Omega}_{ij}
         (\V{e}_{ij} + \V{J}_{ij} \Delta\V{x}_{ij}) \\
      &= \underbrace{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{e}_{ij}}_{c_{ij}}
         + 2 
           \underbrace{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{J}_{ij} }_{\V{b}_{ij}^T}
           \Delta\V{x}_{ij}
         + \Delta\V{x}_{ij}^T
           \underbrace{\V{J}_{ij}^T \BG{\Omega}_{ij} \V{J}_{ij}}_{\V{H}_{ij}}
           \Delta\V{x}_{ij}
    \end{align*}
    where
    - \( c_{ij} \) is a \( 1 \times 1 \) scalar data
    - \( \V{b}_{ij} \) is a \( nk \times 1 \) vector
    - \( \V{H}_{ij} \) is the approximated \( nk \times nk \) 
      Hessian matrix of \( f_{ij} \)
  - Add each \( f_{ij} \) together:
    \begin{align*}
      f(\V{x} \oplus \Delta\V{x})
      &\approx \displaystyle\sum_{i,j} 
         c_{ij} + 2 \V{b}_{ij}^T \Delta\V{x} 
         + \Delta\V{x}^T \V{H}_{ij} \Delta\V{x} \\
      &= c + 2 \V{b}^T \Delta\V{x} + \Delta\V{x}^T \V{H} \Delta\V{x}
    \end{align*}
    where
    - \( c \) is a scalar
    - \( \V{b} \) is a \( nk \times 1 \) vector that is possibly dense
      (with each \( \V{b}_{ij} \) stacked together)
    - \( \V{H} \) is the approximated \( nk \times nk \) 
      Hessian matrix of \( f \)
  - To obtain a best \( \Delta\V{x} \) for each iteration:
    - Gauss-Newton (GN) method: let
      \( \partial f(\V{x} \oplus \Delta\V{x}) / \partial \Delta\V{x} = 0 \)
      \[
         \V{H} \Delta\V{x} = - \V{b}
      \]
    - Levenberg-Marquadt (LM) method: add a damping term based on GN method
      \[
         (\V{H} + \lambda \V{I}) \Delta\V{x} = - \V{b}
      \]
      where \( \lambda \) is a damping factor: as \( \lambda \) becomes higher,
      the direction of \( \Delta\V{x} \) will become nearer that of the 
      negative gradient vector. Therefore it is a mixture of Gauss-Newton and
      gradient descent method.
  - Solve the above linear system of form \( \V{A} \V{x} = \V{b} \), and then
    update \( \V{x} \) for current iteration 
    (\( \V{x} \leftarrow \V{x} \oplus \Delta\V{x} \))
    until the convergence of the cost function \( f \)


## 2. Motion-only BA {#moba}

Motion-only BA is implemented in `Optimizer::PoseOptimization()`{.cpp} function.
In this optimization scheme, only the current camera pose (\( \V{T}_{cw} \)) 
measurement is unknown and is to be optimized with all the 2D keypoint 
observations in image coordinates, meanwhile all the 3D 
map point measurements in world coordinates are fixed during the optimization.

- Procedures:
  (1) Add the vertex and edges into g2o solver (Levenberg-Marquardt)
      - Vertex: camera pose \( \V{T}_{cw} \in SE(3) \) (only for current frame)
        - Pose data is encapsulated in class `g2o::SE3Quat`{.cpp}
        - Vertex is defined in class `g2o::VertexSE3Expmap`{.cpp}
      - Edge: unary edge with only one vertex of the current pose
        - Information matrix: inverse of covariance matrix, which is a diagonal
          matrix, with its diagonal elements being observation variances 
          (\( \sigma^2 \)) determined by scale factor of the ORB image pyramid
          (\( \sigma^2 = (\text{orb scale factor}^{\text{lvl}})^2 \))
  (2) Perform motion-only BA (10 iterations for actual implementation) several
      times (4 times for actual implementation)
      - For each optimization, find edge outliers with \( \chi^2 \) of error 
        larger than a preset threshold (\( \chi^2 = 5.991 \) for actual 
        implementation)
        - \( \chi^2 \): an outlier threshold based on \( \chi^2 \) test 
          assuming the standard deviation of measurement error is 1 pixel 
          for homograhy
      - Exclude the outlier edges for the next optimization, and include the
        once-outlier edges if they are inliers again
      - Count the current number of outliers
  (3) Update optimized pose, and number of map correspondences between 
      2D keypoint observations and 3D map point measurements (the subtraction
      of map correspondences before optimization and the number of outliers of
      last optimization)

### 2.1 Error Function and Jacobian Computation {#mobaefjc}
- Error function (implemented in function
  `g2o::EdgeSE3ProjectXYZOnlyPose::computeError()`{.cpp}):
  \begin{align*}
    \V{e}_{ij}(\V{x}) 
    &= \V{e}(\BG{\xi}) \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} \V{T}_{cw} \V{X}_w \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} \V{X}_c \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} 
                 \mathrm{exp}(\BG{\xi}^\wedge)
                 \V{X}_w
  \end{align*}
  where 
  - \( \V{x}_o \) is the observation (2D undistorted keypoint coordination)
  - \( \V{K} \) is the camera intrinsic matrix
    - \( \V{K}_{2 \times 3}
         = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \end{bmatrix} \)
    - \( \V{K}_{2 \times 4}
         = \begin{bmatrix} \V{K}_{2 \times 3} & \V{0} \end{bmatrix} \)
    - \( f_x \) and \( f_y \): focal length on \( x \)-axis and \( y \)-axis
    - \( c_x \) and \( c_y \): \( (x, y) \) coordination for camera center
  - \( \V{T}_{cw} 
       = \begin{bmatrix} \V{R}_{cw} & \V{t}_{cw} \\ \V{0}^T & 1 \end{bmatrix}
       = \mathrm{exp}(\BG{\xi}^\wedge) \) 
    is the camera pose (world (3D) to camera (3D) transformation matrix),
    where \( \V{R}_{cw} \) is the rotation matrix and \( \V{t}_{cw} \) is the 
    translation vector
  - \( \BG{\xi}^\wedge_{6 \times 1} 
       = \begin{bmatrix} 
           \BG{\phi}_{3 \times 1} \\ \BG{\rho}_{3 \times 1} 
         \end{bmatrix}^\wedge
       = \left((\phi_1, \phi_2, \phi_3, \rho_1, \rho_2, \rho_3)^T\right)^\wedge
       = \begin{bmatrix}
           \BG{\phi}^\wedge & \BG{\rho} \\
           \V{0}^T          & 1
         \end{bmatrix}_{4 \times 4}
       \in \mathfrak{se}(3) \)  
    where
    \( \BG{\phi}^\wedge
       = \begin{bmatrix} 
           0       & -\phi_3 & \phi_2 \\
           \phi_3  & 0       & -\phi_1 \\
           -\phi_2 & \phi_1  & 0
         \end{bmatrix}
    \)
  - \( \V{X}_w = (\bar{\V{X}}_w, 1)^T = (X_w, Y_w, Z_w, 1)^T \) 
    is the 3D robot position in the world
    in homogeneous coordinate
  - \( \V{X}_c = (\bar{\V{X}}_c, 1)^T = (X_c, Y_c, Z_c, 1)^T \) 
    is the 3D robot position relative to camera origin
- Jacobian of the above error function w.r.t. \( \BG{\xi} \) using purturbation
  method & chain rule (implemented in function
  `g2o::EdgeSE3ProjectXYZOnlyPose::linearizeOplus()`{.cpp}):
  \begin{align*}
    \V{J}_{2 \times 6} 
    &= \frac {\partial \V{e}_{2 \times 1}} 
             {\partial \BG{\xi}_{6 \times 1}}
     = \left(
         \frac {\partial \V{e}} {\partial (\V{T}_{cw} \V{X}_w)_{4 \times 1}}
       \right)_{2 \times 4}
       \cdot
       \left(
         \frac {\partial (\V{T}_{cw} \V{X}_w)} {\partial \BG{\xi}}
       \right)_{4 \times 6} \\
    &\approx 
       \frac {\partial \V{e}} {\partial (\V{T}_{cw} \V{X}_w)}
       \cdot
       \frac {\partial 
                \left( 
                  \mathrm{exp}(\delta\BG{\xi}^\wedge) \V{T}_{cw} \V{X}_w 
                \right)}
             {\partial \delta\BG{\xi}} \\
    &= \frac {\partial \V{e}} {\partial \V{X}_c}
       \cdot
       \frac {\partial \left(\mathrm{exp}(\delta\BG{\xi}^\wedge) \V{X}_c\right)}
             {\partial \delta\BG{\xi}}
  \end{align*}
  where
  \begin{align*}
    \frac {\partial \V{e}} {\partial \V{X}_c}
    &= \frac {\partial } {\partial \V{X}_c}
       \left( \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} \V{X}_c \right) \\
    &= \frac {\partial } 
             {\partial \begin{bmatrix} X_c & Y_c & Z_c & 1 \end{bmatrix}^T}
       \left(
         \begin{bmatrix} x_o \\ y_o \end{bmatrix}
         - \frac{1}{Z_c}
           \begin{bmatrix} f_x & 0 & c_x & 0 \\ 0 & f_y & c_y & 0 \end{bmatrix}
           \begin{bmatrix} X_c \\ Y_c \\ Z_c \\ 1 \end{bmatrix}
       \right) \\
    &= \frac {\partial } 
             {\partial \begin{bmatrix} X_c & Y_c & Z_c & 1 \end{bmatrix}^T}
       \left(
         \begin{bmatrix} 
           x_o - f_x \frac{X_c}{Z_c} - c_x \\
           y_o - f_y \frac{Y_c}{Z_c} - c_y
         \end{bmatrix}
       \right) \\
    &= \begin{bmatrix}
         -f_x \frac{1}{Z_c} & 0 & f_x \frac{X_c}{Z_c^2} & 0 \\
         0 & -f_y \frac{1}{Z_c} & f_y \frac{Y_c}{Z_c^2} & 0
       \end{bmatrix}, \tag{5-1} \label{eq5-1}
  \end{align*}
  \begin{align*}
    \frac {\partial \left(\mathrm{exp}(\delta\BG{\xi}^\wedge) \V{X}_c\right)}
          {\partial \delta\BG{\xi}}
    &= \frac {\partial } {\partial \delta\BG{\xi}}
       \left(
         \begin{bmatrix}
           \delta\BG{\phi}^\wedge & \delta\BG{\rho} \\ \V{0}^T & 1
         \end{bmatrix}
         \V{X}_c
       \right) \\
    &= \frac {\partial } 
             {\partial 
              \begin{bmatrix} 
                \delta\phi_1 & \delta\phi_2 & \delta\phi_3 & 
                \delta\rho_1 & \delta\rho_2 & \delta\rho_3 
              \end{bmatrix}^T}
       \begin{bmatrix}
         0 & -\delta\phi_3 & \delta\phi_2 & \delta\rho_1 \\
         \delta\phi_3 & 0 & -\delta\phi_1 & \delta\rho_2 \\
         -\delta\phi_2 & \delta\phi_1 & 0 & \delta\phi_3 \\
         0 & 0 & 0 & 1
       \end{bmatrix}
       \begin{bmatrix} X_c \\ Y_c \\ Z_c \\ 1 \end{bmatrix} \\
    &= \frac {\partial } 
             {\partial 
              \begin{bmatrix} 
                \delta\phi_1 & \delta\phi_2 & \delta\phi_3 & 
                \delta\rho_1 & \delta\rho_2 & \delta\rho_3 
              \end{bmatrix}^T}
       \begin{bmatrix}
         -Y_c \delta\phi_3 + Z_c \delta\phi_2 + \delta\rho_1 \\
          X_c \delta\phi_3 - Z_c \delta\phi_1 + \delta\rho_2 \\
         -X_c \delta\phi_2 + Y_c \delta\phi_1 + \delta\rho_3 \\
         1
       \end{bmatrix} \\
    &= \begin{bmatrix}
            0 &  Z_c & -Y_c & 1 & 0 & 0 \\
         -Z_c &    0 &  X_c & 0 & 1 & 0 \\
          Y_c & -X_c &    0 & 0 & 0 & 1 \\
            0 &    0 &    0 & 0 & 0 & 0 
       \end{bmatrix}.
  \end{align*}
  Therefore,
  \[
     \V{J}_{2 \times 6} = 
     \begin{bmatrix}
       f_x \frac{X_c Y_c}{Z_c^2} & -f_x (1 + \frac{X_c^2}{Z_c^2}) & 
       f_x \frac{Y_c}{Z_c} & 
       -f_x \frac{1}{Z_c} & 0 & f_x \frac{X_c}{Z_c^2} \\
       f_y (1 + \frac{Y_c^2}{Z_c^2}) & -f_y \frac{X_c Y_c}{Z_c^2} &
       -f_y \frac{X_c}{Z_c} &
       0 & -f_y \frac{1}{Z_c} & f_y \frac{Y_c}{Z_c^2}
     \end{bmatrix} \tag{5-2} \label{eq5-2}
  \]
  

## 3. Full/Global BA

Full/Global BA is implemented in function `Optimizer::BundleAdjustment()`{.cpp},
which is called by function `Optimizer::GlobalBundleAdjustment()`{.cpp}.
It is performed in the tracking thread during initial map creation,
and in the loop closing thread during the correction of a detected loop.  

In this optimization scheme, all the 3D map points \( \V{X}_w \) triangulated
and their associated camera poses \( \V{T}_{cw} \) are to be optimized with
2D keypoint observations in image coordinates. An exception during the 
optimization is that the pose of 1st keyframe is fixed, and only the other poses
are to be optimized.

- Procedures:
  (1) Add the vertices and edges into g2o solver (Levenberg-Marquardt)
      - Vertex: 
        - Camera pose \( \V{T}_{cw} \in SE(3) \) for each keyframe
          - Pose data is encapsulated in class `g2o::SE3Quat`{.cpp}
          - Vertex is defined in class `g2o::VertexSE3Expmap`{.cpp}
        - All 3D map points \( \bar{\V{X}}_w \in \V{R}^3 \) 
          (inhomogeneous coordinate) in the current initial map
          - Map point data is encapsulated in class `Eigen::Vector3d`{.cpp}
          - Vertex is defined in class `g2o::VertexSBAPointXYZ`{.cpp}
        - The vertices are sorted: if there're \( n_1 \) poses and \( n_2 \)
          map points, the pose vertices have the index range of 
          \( [0, n_1 - 1] \), and the map point vertices have that of
          \( [n_1, n_1 + n_2 - 1] \)
      - Edge: binary edge with 2 vertices
        - Vertex 0: map point \( \bar{\V{X}}_w \)
        - Vertex 1: the associated camera pose from which the associated map 
          point is triangulated (and the 2D keypoint is observed)
        - Information matrix is the same described [above](#moba)
  (2) Perform full BA for some iterations
      - 20 iterations during initial map creation
      - 10 iterations during loop correction
  (3) Update map point data in the map and camera pose data in each keyframe

### 3.1 Error Function and Jacobian Computation {#fgbaefjc}
- Error function (implemented in function
  `g2o::EdgeSE3ProjectXYZ::computeError()`{.cpp}):
  \begin{align*}
    \V{e}_{ij}(\V{x}) 
    &= \V{e}_{ij}(\V{x}_i, \V{x}_j, \V{z}_{ij}) 
     = \V{e}(\bar{\V{X}}_w, \BG{\xi}) \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} \V{T}_{cw} \V{X}_w \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} \V{X}_c \\
    &= \V{x}_o - \frac{1}{Z_c} \V{K}_{2 \times 4} 
                 \mathrm{exp}(\BG{\xi}^\wedge)
                 \begin{bmatrix} \bar{\V{X}}_w \\ 1 \end{bmatrix}
  \end{align*}
  where related info on the above parameters are described [here](#mobaefjc)
  - The error term is robustified using Huber kernel
    \[
       f_{ij} 
       = \rho_H \left( \sqrt{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{e}_{ij}} \right)
    \]
    where
    \[
       \rho_H (x) = 
       \begin{cases}
         x^2         & \quad \text{if } |x| < b \\
         2b|x| - b^2 & \quad \text{otherwise}
       \end{cases}
    \]
    - \( b = \chi = \sqrt{5.991} \) for monocular SLAM in actual implementation
    - If no robust kernel is used, \( \rho(x) = x^2 \)
  - In g2o, original \( \V{e}_{ij} \) is first computed, and then replaced by
    \( w_{ij} \V{e}_{ij} \) such that
    \[
       (w_{ij} \V{e}_{ij})^T \BG{\Omega}_{ij} (w_{ij} \V{e}_{ij})
       = \rho_H \left( \sqrt{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{e}_{ij}} \right)
    \]
    where
    \[
       w_{ij} 
       = \frac {\sqrt{\rho_H(\|\V{e}_{ij}\|_{\BG{\Omega}_{ij}})}} 
               {\|\V{e}_{ij}\|_{\BG{\Omega}_{ij}}}
       \quad \text{with} \quad
       \|\V{e}_{ij}\|_{\BG{\Omega}_{ij}} 
       = \sqrt{\V{e}_{ij}^T \BG{\Omega}_{ij} \V{e}_{ij}}
    \]
- Jacobian \( \partial \V{e}_{2 \times 1} / \partial \V{x}_{nk \times 1} \) 
  computation
  - Only need to compute 2 sub Jacobian matrices
    \( \partial \V{e} / \partial \bar{\V{X}}_w \) and
    \( \partial \V{e} / \partial \BG{\xi} \)
    to form the full \( 2 \times nk \) Jacobian
  - Jacobian \( \partial \V{e} / \partial \BG{\xi} \): the result has been
    shown in \eqref{eq5-2}
  - Jacobian \( \partial \V{e} / \partial \bar{\V{X}}_w \):
    \begin{align*}
      \V{J}_{2 \times 3}
      &= \frac {\partial \V{e}_{2 \times 1}} 
               {\partial \bar{\V{X}}_{w, 3 \times 1}} \\
      &= \frac {\partial \V{e}} {\partial \bar{\V{X}}_c}
         \frac {\partial \bar{\V{X}}_c} {\partial \bar{\V{X}}_w} \\
      &= \frac {\partial \V{e}} {\partial \bar{\V{X}}_c}
         \frac {\partial (\V{R}_{cw} \bar{\V{X}}_w + \V{t}_{cw})}
               {\partial \bar{\V{X}}_w} \\
      &= \frac {\partial \V{e}} {\partial \bar{\V{X}}_c} \V{R}_{cw}
    \end{align*}
    From \eqref{eq5-1},
    \[
       \frac {\partial \V{e}} {\partial \bar{\V{X}}_c}
       = \begin{bmatrix}
           -f_x \frac{1}{Z_c} & 0 & f_x \frac{X_c}{Z_c^2} \\
           0 & -f_y \frac{1}{Z_c} & f_y \frac{Y_c}{Z_c^2} 
         \end{bmatrix}
    \]
    

## 4. Local BA

Local BA is performed in the local mapping thread after a new keyframe
is added to the map and all the map point data related to this new keyframe
is updated. It is implemented in function 
`Optimizer::LocalBundleAdjustment()`{.cpp}.

In this optimization scheme, the new keyframe with all its connected keyframes
are to be optimized, that is, all the corresponding camera poses of the 
keyframes, and all the map points observed by these keyframes are to be
optimized. All other keyframes that can observe the map points above, but are
not connected with the new keyframe, are also included in the optimization
scheme, but with their camera poses fixed throughout the optimization.

The procedures are as follows:

(1) Add the current new keyframe and its connected keyframes into a list of
    local keyframes;
(2) Add all the map points that can be observed by the local keyframes into
    a list of local map points;
(3) Add all the keyframes that can observe the local map points but are not
    connected to the current new keyframe into a list of local fixed keyframes;
(4) Add vertices and edges into g2o solver (Levenberg-Marquardt):
    - Vertex:
      - Camera pose for each local keyframe, and fixed camera pose for 
        each local fixed keyframe;
        - Pose data is encapsulated in class `g2o::SE3Quat`{.cpp}
        - Vertex is defined in class `g2o::VertexSE3Expmap`{.cpp}
      - All 3D map points in the local map point list
        - Map point data is encapsulated in class `Eigen::Vector3d`{.cpp}
        - Vertex is defined in class `g2o::VertexSBAPointXYZ`{.cpp}
    - Edge: binary edge with 2 vertices
        - Vertex 0: map point
        - Vertex 1: the associated camera pose from which the associated map 
          point is triangulated (and the 2D keypoint is observed)
        - Information matrix is the same described [above](#moba)
(5) Perform local BA optimization for some iterations;
    - 5 iterations in actual implementation
(6) After the optimization, exclude outlier edges from the g2o optimization
    graph whose \( \chi^2 \) error is larger than the \( \chi^2 \) threshold
    5.991 (see [here](#pc-tp) for the explanation of the threshold);
(7) Perform another round of local BA optimization for some iterations;
    - 10 iterations in actual implementation
(8) For all the edges including the previously outlier ones, find all
    edges with their \( \chi^2 \) error larger than the \( \chi^2 \) threshold,
    and remove all the map points that are associated to
    these outlier edges from the associated keyframes. The observation 
    info of the map points on these keyframes are also removed;
(9) Update camera poses with optimized ones;
(10) Update map point coordinates with optimized ones, and then
     update the normal and depth info of each map point.

### 4.1 Error Function and Jacobian Computation

Error function and Jacobian computation are the same
with those in global BA procedure described [above](#fgbaefjc).


\newpage

