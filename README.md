# Inverse-kinematics-project-based-on-the-Abb_Irb120.urdf-model
This project implements forward and numerical inverse kinematics for the ABB IRB120 6-axis industrial robot virtual model using the C programming language. 
For testing the inverse kinematics solver, I execute it through MATLAB through mexFunction() by icluding "mex.h" library in the main file. 

The project was developed primarily as a implementation of robot kinematics without relying on MATLAB's built-in inverse kinematics solver.

The main features are:
  [x] 6-DOF ABB IRB120 forward kinematics
  [x] Numerical geometric Jacobian calculation
  [x] Position and orientation error calculation
  [x] Damped Least Squares imlementation in inverse kinematics
  [x] Custom 6x6 Gaussian elimination solver
  [x] MATLAB MEX interface
  [x] Cartesian trajectory testing
  [x] Configurable IK convergence options and step limits

# Project Structure
## Forward kinematics
In this section, I represent each robot joint as homogeneous 4x4 transformation:
```iecst
Link1(double q, Matrix_4x4 *T)
```
, where 'q' is joint corresponding angle and 'T' is a pointer to transformation matrix.

Using matrix multiplication defined function 'multiply_matrix', I get the resulting 4x4 matrix that contains the end-effector position and orientation.
Then I use function 'robot_pose' that converts the transformation matrix to [x, y, z, roll, pitch, yaw] using the ZYX - Euler convention. 

## Rotation mathematics
Since this is a inverse kinematics engine, I also need to give an target, but it would be nearly impossible to write in 4x4 transformation matrix. For that, I define function 'pose_to_transform', that converts the given [x, y, z, roll, pitch, yaw] to transformation matrix.

For orientation error between two transformation matrices, I compute using function 'rotation_multiply_transpose()':
```iecst
for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            R[i][j] = 0.0;
            for (k = 0; k < 3; ++k)
                R[i][j] += A->m[i][k] * B->m[j][k];
        }
    }
}
```
, and here switching 'B->m[k][j]' to 'B->m[j][k]', I am multiplying: 'R_error = R_target * R_current^T'.

Then, I represent difference between TWO 3D - orientations as a 3-ELEMENT vector for Jacobian matrix using function 'rotation_log_vector' which converts a 3x3 rotation error matrix into a three-element rotation vector. Thisspecial handling is used for:
[x] very small rotation angles
[x] rotations close to 180 degrees

Then using function 'transform_error' I construct the full error[6] six-element vector for the 6-axis (6DOF) robot.

## ABB_IRB120_inverse_kinematics

Contains the numerical inverse kinematics algorithm. The main function:
```iecst
int inverseKinematics(
    const Pose *targetPose,
    double jointAngles[6],
    const IK_Options *options,
    IK_Result *result)
```
, inputs are:
[x] Desired Crtesian pose
[x] Initial joint configuration
[x] IK solver options
, where outputs are:
[x] Calculated joint angle vector q[6]
[x] Convergence information
[x] Final Cartesian error

## Numrical Jacobian
The robot geometric jacobian is calculated numerically using function:
```iecst
void numericalJacobian(
    const double jointAngles[6],
    double numericalStep,
    Matrix_6x6 *J)
```
The Jacobian itself is: "a mathematical map that translates small changes in a robot's joint angles into corresponding movements of its end-effector" , where here the end-effector is the robot tool attached to the end of the arm.

For every joint:
```iecst
qp[joint] += h;
qm[joint] -= h;
```
, I add/remove a small step. Forward kinematics is calculated for both configurations. Therefore the linear Jacobian component is calculated using a centered finite difference:
```iecst
J->m[0][joint] = (Tp.m[0][3] - Tm.m[0][3])/(2.0*h);
        J->m[1][joint] = (Tp.m[1][3] - Tm.m[1][3])/(2.0*h);
        J->m[2][joint] = (Tp.m[2][3] - Tm.m[2][3])/(2.0*h);
```
, and the rotation matrix is converted into a rotation vector and divided by '2*h':
```iecst
for (row = 0; row < 3; ++row)
    J->m[row + 3][joint] = rotDelta[row]/(2.0*h);
```
In the final for 6 - joints I perform 12 computations. The returned variable is a 6x6 matrix:
[linear velocity]
[linear velocity]
[linear velocity]
[angular velocity]
[angular velocity]
[angular velocity]

## Damped Least Squares

The solve uses a damped-least-squares approach. At every iteration:
1) Calculating Current forward kinematics
2) Calculating Cartesian position and orientation errors
3) Error-vector magnitudes are limited to prevent excessively large Cartesian steps
4) A numerical 6x6 Jacobian is generated
5) Position and orientation weighting is applied
6) The DLS system is constructed: A = J * J^T + lambda^2 * I
7) The linear system is solved: A * y = error
8) Joint corrections are calculated: dq = J^T * y
9) Each joint correction is limited by maxJointStep
10) The new joint configuration is calculated: q = q + dq

The procedure repeats until the Cartesian position and orientation errors satisfy the configured tolerances or the maximum iteration count is reached.

## Gaussian solver
Implements a custom linear solver for the 6x6 DLS approach. The functionn solve_6x6 first copies the supplied matrix and right-hand-side vector so the original input data is not modified. 

Then I call function 'forward_elimination()'. Here, Forward elimination uses partial pivoting by selecting the largest available pivot in each column. Very small pivots are eliminated to prevent solving singular or numerically unstable systems.

After function 'forward_elimination', I call 'back_substitution' which eliminates the smallest pivots under the main diagonal.

## IK configuration
Default solver parameters are defined using function 'ik_default_options':
```iecst
void ik_default_options(IK_Options *options)
{
    options->maxIterations = 300;
    options->positionTolerance = 1e-5;
    options->orientationTolerance = 1e-4;
    options->damping = 1e-2;
    options->numericalStep = 1e-6;
    options->maxCartesianStep = 0.010;   /* 10 mm */
    options->maxOrientationStep = 0.050; /* about 2.9 deg */
    options->maxJointStep = 0.050;       /* about 2.9 deg */
    options->positionWeight = 1.0;
    options->orientationWeight = 0.25;
}
```
These limits control the numerical IK iteration and should not be interpreted as physical robot velocity or acceleration limits.

## MATLAB MexFunction
The C implementation is compiled as a MATLAB MEX module:
```iecst
void mexFunction(
    int nlhs,              /* Number of expected mxArray output arguments, specified as an integer */
    mxArray *plhs[],       /* Array of pointers to the expected mxArray output arguments */
    int nrhs,              /* Number of input arguments */
    const mxArray *prhs[]) /* Array of pointers to the mxArray input arguments */
```
In MATLAB Window to compile my C implementation in .mexw64, I must write:
```iecst
mex CallMexFunction.c Rotation_math.c Gaussian_solver.c ABB_IRB120_forward_kinematics.c ABB_IRB120_inverse_kinematics.c
```
To the C solver, MATLAB sends:
[x] previous/initial configuration
[x] target Cartesian pose

The Mex function the returns:
[x] Calculted joint angles
[x] Resultig transformation amtrix
[x] Convergence information

## Forward kinematics validation
The C forward kinematics implementation was compared against MATLAB Robotics System Toolbox using the ABB IRB120 URDF model.
For example, for joint configuration in MATLAB:
```iecst
q = [0, pi/4, pi/5, 0, 0, 0];
```
, the calculated transformation matrix using C implementation was:
```iecst
Transformation matrix from C mex
    0.1564         0    0.9877    0.3186
         0    1.0000         0         0
   -0.9877         0    0.1564    0.1225
         0         0         0    1.0000
```
, and using MATLAB Robot System Toolbox as validity check:
```iecst
Transformation matrix from URDF usng FUN "getTransform"
    0.1564         0    0.9877    0.3186
         0    1.0000         0         0
   -0.9877         0    0.1564    0.1225
         0         0         0    1.0000
```
, therefore the computed error between both results was:
```iecst
Error = 5.551115123125783e-17
```

## Trajectory testing
MATLAB is used to generate a sequence of Cartesian target positions. For each target point:

1) The previous valid joint configuration is supplied as the initial IK guess
2) The C MEX inverse kinematics function is executed
3) Joint angles and convergence information are returned
4) Successfully calculated joint configurations can be visualized using the MATLAB rigidBodyTree model

## Current limitations


   




