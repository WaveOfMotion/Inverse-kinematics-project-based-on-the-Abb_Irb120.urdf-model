#include <math.h>
#include <string.h>
#include "robot_constants.h"
#include "ABB_IRB120_forward_kinematics.h"

double clamp_scalar(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double norm3(double v[3]) // sqrt(x^2 + y^2 + z^2)
{
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/*
 * Convert ZYX Euler convention to transformation matrix to compute position and orientation error
 */
void pose_to_transform(const Pose *pose, Matrix_4x4 *T)
{
    /* ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll) */
    const double cr = cos(pose->roll);
    const double sr = sin(pose->roll);
    const double cp = cos(pose->pitch);
    const double sp = sin(pose->pitch);
    const double cy = cos(pose->yaw);
    const double sy = sin(pose->yaw);

    T->m[0][0] = cy*cp;
    T->m[0][1] = cy*sp*sr - sy*cr;
    T->m[0][2] = cy*sp*cr + sy*sr;
    T->m[0][3] = pose->x;

    T->m[1][0] = sy*cp;
    T->m[1][1] = sy*sp*sr + cy*cr;
    T->m[1][2] = sy*sp*cr - cy*sr;
    T->m[1][3] = pose->y;

    T->m[2][0] = -sp;
    T->m[2][1] = cp*sr;
    T->m[2][2] = cp*cr;
    T->m[2][3] = pose->z;

    T->m[3][0] = 0.0;
    T->m[3][1] = 0.0;
    T->m[3][2] = 0.0;
    T->m[3][3] = 1.0;
}

/* 
 * Create orientation-error-matrix: R_err = R_target * R_current^T , where R^(-1) = R^T 
 */
void rotation_multiply_transpose(
    const Matrix_4x4 *A,
    const Matrix_4x4 *B,
    double R[3][3])
{
    /* R = A.R * B.R^T */
    int i, j, k;

    for (i = 0; i < 3; ++i)
    {
        for (j = 0; j < 3; ++j)
        {
            R[i][j] = 0.0;
            for (k = 0; k < 3; ++k)
                R[i][j] += A->m[i][k] * B->m[j][k]; /* ---> here B->m[k][j] is R_current , but B->m[j][k] is R_current^T */
        }
    }
}

/*
 * Represent difference between TWO 3D-ORIENTATIONS as a 3-ELEMENT vector for JACOBIAN MATRIX
 */
void rotation_log_vector(const double R[3][3], double w[3])
{

    /* ---> Sum the rotation-error matrix diagonal elements to extract the angle: trace(R) = 1 + 2cos(theta) */
    const double trace = R[0][0] + R[1][1] + R[2][2];

    /* ---> Retrieve the angle and clamp angle in bounds */
    const double cosTheta = clamp_scalar(0.5*(trace - 1.0), -1.0, 1.0);

    /* ---> Retrieve the value of rotation angle in radians */
    const double theta = acos(cosTheta);

    /* ---> When angle is extremely small = orientation error close to zero ---> use small-angle-approximation*/
    if (theta < 1e-8)
    {
        /* ---> R_21 - R_12 = 2sin(theta) * u_x */
        w[0] = 0.5*(R[2][1] - R[1][2]);

        /* R_02 - R_20 = 2sin(theta) * u_y */
        w[1] = 0.5*(R[0][2] - R[2][0]);

        /* R_10 - R_01 = 2sin(theta) * u_z */
        w[2] = 0.5*(R[1][0] - R[0][1]);
        return;
    }

    /* ---> If the rotation angle is very closee to 180 degrees */
    if (PI - theta < 1e-5)
    {
        /* Compute the vector of each axis */
        double axis[3];
        axis[0] = sqrt(fmax(0.0, 0.5*(R[0][0] + 1.0)));
        axis[1] = sqrt(fmax(0.0, 0.5*(R[1][1] + 1.0)));
        axis[2] = sqrt(fmax(0.0, 0.5*(R[2][2] + 1.0)));

        /* Rotation-error-matrix elements is close to zero -> angle close to PI -> switch axis sign */
        if (R[2][1] - R[1][2] < 0.0) { 
            axis[0] = -axis[0]; 
        }
        if (R[0][2] - R[2][0] < 0.0) {
            axis[1] = -axis[1]; 
        }
        if (R[1][0] - R[0][1] < 0.0) {
            axis[2] = -axis[2]; 
        }

        /* ---> |w| = theta , where w/|w| = rotation axis , where direction of w -> axis, but length of w -> angle */
        w[0] = theta*axis[0];
        w[1] = theta*axis[1];
        w[2] = theta*axis[2];
        return;
    }

    /* ---> Default rotation-error-matrix rotation-error element computations */
    {
        const double scale = theta/(2.0*sin(theta));
        w[0] = scale*(R[2][1] - R[1][2]);
        w[1] = scale*(R[0][2] - R[2][0]);
        w[2] = scale*(R[1][0] - R[0][1]);
    }
}

void transform_error(
    const Matrix_4x4 *current,
    const Matrix_4x4 *target,
    double error[6])
{
    double Rerr[3][3];
    double w[3];

    /* ---> Compute position-error */
    error[0] = target->m[0][3] - current->m[0][3];
    error[1] = target->m[1][3] - current->m[1][3];
    error[2] = target->m[2][3] - current->m[2][3];

    /* ---> Base-frame orientation error: R_err = R_target * R_current^T */
    rotation_multiply_transpose(target, current, Rerr);
    rotation_log_vector(Rerr, w);

    error[3] = w[0];
    error[4] = w[1];
    error[5] = w[2];
}