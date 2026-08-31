#ifndef ABB_IRB120_inverse_kinematics_H
#define ABB_IRB120_inverse_kinematics_H

#include "robot_constants.h"
#include "Rotation_math.h"
#include "Gaussian_solver.h"
#include "ABB_IRB120_forward_kinematics.h"

typedef struct
{
    double m[6][6];
} Matrix_6x6;

typedef struct
{
    int maxIterations;
    double positionTolerance;   /* metres */
    double orientationTolerance;/* radians */
    double damping;             /* DLS lambda */
    double numericalStep;       /* radians */
    double maxCartesianStep;    /* metres per IK iteration */
    double maxOrientationStep;  /* radians per IK iteration */
    double maxJointStep;        /* radians per IK iteration */
    double positionWeight;
    double orientationWeight;
} IK_Options;

typedef struct
{
    int converged;
    int iterations;
    double positionError;
    double orientationError;
} IK_Result;

void clamp_norm3(double v[3], double maxNorm);

void ik_default_options(IK_Options *options);

/*
 * Numerical geometric Jacobian:
 * rows 0..2: linear velocity in base frame [m/rad]
 * rows 3..5: angular velocity in base frame [rad/rad]
 */
void numericalJacobian(
    const double jointAngles[6],
    double numericalStep,
    Matrix_6x6 *J);

/*
 * Solve one complete endpoint IK problem.
 * jointAngles is both the initial guess and the returned solution.
 */
int inverseKinematics(
    const Pose *targetPose,
    double jointAngles[6],
    const IK_Options *options,
    IK_Result *result);

#endif
