#ifndef ROTATION_MATH_H
#define ROTATION_MATH_H

#include <math.h>
#include <string.h>
#include "ABB_IRB120_forward_kinematics.h"

double clamp_scalar(double x, double lo, double hi);
double norm3(double v[3]);
void pose_to_transform(const Pose *pose, Matrix_4x4 *T);

/*
 * Convert ZYX Euler convention to transformation matrix to compute position and orientation error
 */
void pose_to_transform(const Pose *pose, Matrix_4x4 *T);
void rotation_multiply_transpose(
    const Matrix_4x4 *A,
    const Matrix_4x4 *B,
    double R[3][3]);
/*
 * Represent difference between TWO 3D-ORIENTATIONS as a 3-ELEMENT vector for JACOBIAN MATRIX
 */
void rotation_log_vector(const double R[3][3], double w[3]); 
/*
 * Output full 6x1 error vector
 */
void transform_error(
    const Matrix_4x4 *current,
    const Matrix_4x4 *target,
    double error[6]);
    
#endif