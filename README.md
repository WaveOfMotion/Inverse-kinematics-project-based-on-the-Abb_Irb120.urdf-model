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

