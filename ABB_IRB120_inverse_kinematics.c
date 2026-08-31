#include <math.h>
#include <string.h>
#include "robot_constants.h"
#include "Rotation_math.h"
#include "Gaussian_solver.h"
#include "ABB_IRB120_forward_kinematics.h"
#include "ABB_IRB120_inverse_kinematics.h"

void clamp_norm3(double v[3], double maxNorm)
{
    double n = norm3(v);

    if (maxNorm <= 0.0)
        return;

    if (n > maxNorm && n > 0.0)
    {
        double scale = maxNorm / n;
        v[0] *= scale;
        v[1] *= scale;
        v[2] *= scale;
    }
}

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

void numericalJacobian(
    const double jointAngles[6],
    double numericalStep,
    Matrix_6x6 *J)
{
    int joint, row;

    /* variable = condition ? value_if_true : value_if_false */
    double h;
    if (numericalStep > 0.0) {
        h = numericalStep;
    }
    else { 
        h = 1e-6;
    }

    for (joint = 0; joint < 6; ++joint)
    {
        double qp[6];
        double qm[6];
        Matrix_4x4 Tp;
        Matrix_4x4 Tm;
        double Rrel[3][3];
        double rotDelta[3];

        memcpy(qp, jointAngles, 6*sizeof(double));
        memcpy(qm, jointAngles, 6*sizeof(double));

        qp[joint] += h;
        qm[joint] -= h;

        forward_kinematics(qp, &Tp);
        forward_kinematics(qm, &Tm);

        J->m[0][joint] = (Tp.m[0][3] - Tm.m[0][3])/(2.0*h);
        J->m[1][joint] = (Tp.m[1][3] - Tm.m[1][3])/(2.0*h);
        J->m[2][joint] = (Tp.m[2][3] - Tm.m[2][3])/(2.0*h);

        /*
         * Centered finite-difference angular column.
         * R_plus * R_minus^T corresponds to approximately 2*h*omega.
         */
        rotation_multiply_transpose(&Tp, &Tm, Rrel);
        rotation_log_vector(Rrel, rotDelta);

        for (row = 0; row < 3; ++row)
            J->m[row + 3][joint] = rotDelta[row]/(2.0*h);
    }
}

int inverseKinematics(
    const Pose *targetPose,
    double jointAngles[6],
    const IK_Options *options,
    IK_Result *result)
{
    IK_Options defaults;
    Matrix_4x4 targetT;
    int iteration;

    if (options == 0)
    {
        ik_default_options(&defaults);
        options = &defaults;
    }

    pose_to_transform(targetPose, &targetT);

    result->converged = 0;
    result->iterations = 0;
    result->positionError = HUGE_VAL;
    result->orientationError = HUGE_VAL;

    for (iteration = 0; iteration < options->maxIterations; ++iteration)
    {
        Matrix_4x4 currentT;
        Matrix_6x6 J;
        double fullError[6];
        double command[6];
        double weightedJ[6][6];
        double weightedError[6];
        double A[6][6];
        double y[6] = {0,0,0,0,0,0};
        double dq[6] = {0,0,0,0,0,0};
        double position_error[3], rotation_error[3];
        int i, j, k;

        forward_kinematics(jointAngles, &currentT);
        transform_error(&currentT, &targetT, fullError);

        position_error[0] = fullError[0];
        position_error[1] = fullError[1];
        position_error[2] = fullError[2];
        rotation_error[0] = fullError[3];
        rotation_error[1] = fullError[4];
        rotation_error[2] = fullError[5];

        result->positionError = norm3(position_error);
        result->orientationError = norm3(rotation_error);
        result->iterations = iteration;

        if (result->positionError <= options->positionTolerance &&
            (options->orientationWeight <= 0.0 ||
             result->orientationError <= options->orientationTolerance))
        {
            result->converged = 1;
            return 1;
        }

        /*
         * This is the "divide a far target into checkpoints" operation:
         * clamp the Cartesian and rotational error-vector norms.
         */
        clamp_norm3(position_error, options->maxCartesianStep);
        clamp_norm3(rotation_error, options->maxOrientationStep);

        command[0] = position_error[0];
        command[1] = position_error[1];
        command[2] = position_error[2];
        command[3] = rotation_error[0];
        command[4] = rotation_error[1];
        command[5] = rotation_error[2];

        numericalJacobian(jointAngles, options->numericalStep, &J);

        for (i = 0; i < 6; ++i)
        {
            double weight;
            
            if (i < 3) {
                weight = options->positionWeight; 
            }
            else {
                weight = options->orientationWeight; 
            }

            weightedError[i] = weight*command[i];
            for (j = 0; j < 6; ++j)
                weightedJ[i][j] = weight*J.m[i][j];
        }

        /* A = Jw*Jw^T + lambda^2*I */
        for (i = 0; i < 6; ++i)
        {
            for (j = 0; j < 6; ++j)
            {
                A[i][j] = 0.0;
                for (k = 0; k < 6; ++k)
                    A[i][j] += weightedJ[i][k]*weightedJ[j][k];
            }
            A[i][i] += options->damping * options->damping;
        }

        /* y = inverse(A)*weightedError, computed by solving A*y=e. */
        /* This line means: try to solve function solve_6x6, if can't -> return */
        if (solve_6x6(A, weightedError, y) == 1)
        {
            return 0;
        }

        /* dq = Jw^T*y */
        for (i = 0; i < 6; ++i)
        {
            for (j = 0; j < 6; ++j)
                dq[i] += weightedJ[j][i]*y[j];

            dq[i] = clamp_scalar(
                dq[i],
                -options->maxJointStep,
                 options->maxJointStep);

            jointAngles[i] += dq[i];
        }
    }

    result->iterations = options->maxIterations;
    return 0;
}